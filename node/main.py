"""LoPy4/FiPy LoRa node firmware for Pipebots Sprint 2

This is the MicroPython firmware for the LoPy4/FiPy node that will be integrat-
ed with the SprintBot. The node is supposed to take WLAN RSSI measurements
every 60 seconds, increment a packet counter, and send the data over a LoRa
link to one, or more, of the LoRa gateways that will sit in specific manholes.

This file needs to be uploaded/flashed to a LoPy4/FiPy board together with the
lib/crc32.py and lib/node_cfg.json files. The latter one needs customising
on the day before running the experiment.
"""

import gc
import pycom
import machine
import micropython

import json
import time
import struct
import socket
import binascii

from network import WLAN
from network import LoRa

from lib import crc32


# * For diagnostic messages
DEBUG_MODE = micropython.const(0x01)

# * LoPy4/FiPy use an RGB LED, where each colour is 0 - 255
COLOUR_RED = micropython.const(0xFF0000)
COLOUR_GREEN = micropython.const(0x00FFF00)
COLOUR_BLUE = micropython.const(0x0000FF)
COLOUR_OFF = micropython.const(0x000000)

MAX_PKG_LEN = micropython.const(0x100)
message_count = 0x01


def flash_led(colour: int = 0x0, period: float = 0.25, times: int = 3) -> None:
    """Flash the on-board RGB LED in a particular pattern

    A function useful for debugging and status indication purposes. Flashes
    the on-board RGB LED in a pattern specified by the arguments, before
    leaving it constantly on.

    Args:
        colour: The colour to be used, as a combination of red, green and blue
                intensities. A 3-byte number, one byte per colour.
        period: The time in milliseconds between consecutive flashes.
        times: How many times to turn the LED on and off before settling on
               the solid colour.

    Returns:
        None

    Raises:
        Nothing
    """

    for _ in range(times):
        pycom.rgbled(colour)
        time.sleep(period)
        pycom.rgbled(COLOUR_OFF)
        time.sleep(period)

    pycom.rgbled(COLOUR_OFF)


def open_lora_socket(lora_config):
    """Initialises the LoRa radio and opens a LoRa.RAW socket

    There is a bit of a fudge because JSON files cannot hold values such as
    `True` or `False`, so we have to convert those strings to Python's bool-
    ean constants. A similar hack, using `getattr()` is utilised to convert
    from string constants to the `LoRa` module numeric constants.

    Note:
        The socket is opened in blocking mode. Potentially subject to change
        in future releases. Blocking mode waits for data to be transmitted
        before continuing execution.

    Args:
        lora_config: A `Dict` with all the necessary parameters, loaded from
                     a JSON file.

    Returns:
        A LoRa object and a LoRa socket. The former can be used to control the
        LoRa radio dynamically, i.e. change bandwidth, spreading factor,
        centre frequency, etc., while the latter is used to send and receive
        `bytes` data.

    Raises:
        Nothing
    """

    if "true" == lora_config["tx_iq"].lower():
        tx_iq = True
    else:
        tx_iq = False

    if "true" == lora_config["rx_iq"].lower():
        rx_iq = True
    else:
        rx_iq = False

    lora_obj = LoRa(
        mode=getattr(LoRa, lora_config["mode"]),
        region=getattr(LoRa, lora_config["region"]),
        bandwidth=getattr(LoRa, lora_config["bandwidth"]),
        coding_rate=getattr(LoRa, lora_config["coding_rate"]),
        frequency=lora_config["frequency"],
        tx_power=lora_config["tx_power"],
        sf=lora_config["sf"],
        tx_iq=tx_iq,
        rx_iq=rx_iq,
    )

    lora_socket = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
    lora_socket.setblocking(True)

    return (lora_obj, lora_socket)


def calc_lora_pkg_size(lora_pkg_cfg, payload):
    """Calculates the size of a LoRa packet to be transmitted

    This function is used to determine how many bytes a particular RAW LoRa
    packet will consist of. The function uses `struct.calcsize()` and format
    strings specified in the JSON config file to determine the overall length.

    Note:
        This function has been renamed to reflect its new use. It now supports
        variable-length payloads and is more universal.

    Args:
        lora_pkg_cfg: A `Dict` consisting of format strings indicating how
                      many bytes each field in the packet is.
        payload: `bytes` or `bytearray` with the data to be send.

    Return:
        An integer number indicating the overall length of the LoRa packet
        in bytes.

    Raises:
        Nothing
    """

    pkg_length = 0
    msg_len = len(payload)

    for field in lora_pkg_cfg:
        if "info_payload" == field:
            pkg_length += struct.calcsize(
                lora_pkg_cfg[field].format(msg_length=msg_len)
            )
        else:
            pkg_length += struct.calcsize(lora_pkg_cfg[field])

    return pkg_length


def construct_lora_pkg(lora_pkg_cfg, lora_msg_buf, message_count, payload):
    """Packs experiment data in a buffer to be sent over a LoRa link

    Some of the data changes between each LoRa packet, and therefore needs to
    be updated. This function reuses the same `bytearray` object and writes
    the new values at the correct offsets.

    Note:
        This function will likely change as more packet values become
        variable in future iterations of the protocol.

    Args:
        lora_pkg_cfg: A `Dict` consisting of format strings indicating how
                      many bytes each field in the packet is.
        lora_msg_buf: A `bytearray` or a `memoryview` of one, which holds the
                      data to be transmitted. Needs to be correct size to
                      accommodate the `payload`.
        message_count: An `int`, currently 32-bit wide, serves as transmit
                       packet counter.
        payload: A `bytes` or a `bytearray` object with all the measurements.

    Returns:
        The payload and the rest of the data are packed in-place in the
        `bytearray` buffer to reduce the amount of object creation and
        destruction, as well as to minimise overall memory requirements.

    Raises:
        Nothing
    """

    struct.pack_into(
        lora_pkg_cfg["message_cnt"],
        lora_msg_buf,
        MSG_CNT_OFFSET,
        message_count,
    )

    msg_len = len(payload)
    struct.pack_into(
        lora_pkg_cfg["message_len"], lora_msg_buf, MSG_LEN_OFFSET, msg_len
    )

    struct.pack_into(
        lora_pkg_cfg["info_payload"].format(msg_length=msg_len),
        lora_msg_buf,
        PAYLOAD_OFFSET,
        *payload
    )

    # * The CRC32 checksum is calculated over the entire packet, including
    # * the packet counter. It is added at the end of the packet, making it
    # * reuseable in the future where the payload length will be variable.
    payload_checksum = crc32.crc32_compute(
        lora_msg_buf[: PAYLOAD_OFFSET + msg_len]
    )
    struct.pack_into(
        lora_pkg_cfg["checksum"],
        lora_msg_buf,
        PAYLOAD_OFFSET + msg_len,
        payload_checksum,
    )


def measure_wlan_rssi(wlan_obj, wlan_ssid, missing_value):
    """Measures the RSSI of a specified WLAN SSID

    This is used as a rudimentary propagation measurement experiment. We meas-
    ure the Received Signal Strength Indicator (RSSI) of a specified WLAN, as
    well as the channel that the SSID is advertised on and the BSSID, or the
    MAC of the AP. We do not have to be connected to the WLAN to take those
    measurements.

    Args:
        wlan_obj: A `WLAN` class giving access to the WLAN radio on-board and
                  the results from scanning for available WLANs.
        wlan_ssid: A `str` representing the WLAN SSID we are interested in.
        missing_value: An `Int` value, which is returned in case the specified
                       WLAN SSID cannot be detected.

    Returns:
        A sequence of `bytes` which contains either the measured data or three
        copies of the `missing_value` parameter.

    Raises:
        Nothing
    """

    wlan_networks = wlan_obj.scan()

    for wlan_network in wlan_networks:
        if wlan_ssid in wlan_network:
            rssi = wlan_network.rssi
            channel = wlan_network.channel
            bssid = wlan_network.bssid

            meas_data = (
                rssi.to_bytes(1, "little", True)
                + channel.to_bytes(1, "little", False)
                + bssid
            )
            break
    else:
        meas_data = bytes([missing_value, missing_value, missing_value])

    return meas_data


def run_experiment(lora_msg_buf_ptr):
    """Main function of the Sprint 2 LoRa experiment - node

    This function is called every `meas_interval_sec` number of seconds, as
    defined in the `node_cfg` JSON file. It explicitly defines the variables
    it is going to use as global to help traceability and debugging. The body
    of the function takes a WLAN RSSI and channel number measurement, packs
    those into a RAW LoRa packet, and transmits it over the air.

    Note:
        The callback mechanism allows for a single argument to be passed to
        the function, which is why the `global` definitions are needed.

    Args:
        lora_msg_buf_ptr: A `memoryview` into a `bytearray` object, which is
                          the data to be transmitted.

    Returns:
        None

    Raises:
        Nothing
    """

    global lora_socket
    global node_cfg
    global message_count

    meas_data = measure_wlan_rssi(
        wlan_obj, node_cfg["wlan_ssid"], node_cfg["meas_NA"]
    )
    if DEBUG_MODE:
        print(meas_data)

    construct_lora_pkg(
        node_cfg["lora_pkg_format"], lora_msg_buf_ptr, message_count, meas_data
    )

    pkg_length = calc_lora_pkg_size(node_cfg["lora_pkg_format"], meas_data)

    if DEBUG_MODE:
        print(lora_msg_buf[:pkg_length])

    lora_socket.send(lora_msg_buf_ptr[:pkg_length])

    message_count += 1

    # * Visual feedback that the node is doing something
    flash_led(COLOUR_GREEN, 0.1, 1)

    machine.idle()


def validate_config(node_cfg):
    """Perform validation of JSON config file

    This does a quick check if all necessary fields were present in the JSON
    configuration file. There is also a series of checks if the values for the
    LoRa configuration are correct. Perhaps overkill, but better safe than
    sorry.

    Args:
        node_cfg: A `Dict` with the configuration settings for the LoRa node

    Returns:
        True if all checks pass

    Raises:
        KeyError: If at least one configuration parameter was not present
        ValueError: If there is an incorrect value for the LoRa configuration,
                    or if any of the other parameters were empty/null.
    """

    _MAIN_FIELDS = [
        "node_id",
        "wlan_ssid",
        "meas_interval_sec",
        "meas_NA",
        "lora_config",
        "lora_pkg_format",
    ]

    _LORA_FIELDS = [
        "mode",
        "region",
        "frequency",
        "tx_power",
        "bandwidth",
        "sf",
        "coding_rate",
        "tx_iq",
        "rx_iq",
    ]

    _LORA_PKG_FIELDS = [
        "node_id",
        "message_cnt",
        "message_len",
        "info_payload",
        "checksum",
    ]

    for field in _MAIN_FIELDS:
        if field not in node_cfg:
            raise KeyError
        if node_cfg[field] is None:
            raise ValueError

    for field in _LORA_FIELDS:
        if field not in node_cfg["lora_config"]:
            raise KeyError
        if node_cfg["lora_config"][field] is None:
            raise ValueError

    for field in _LORA_PKG_FIELDS:
        if field not in node_cfg["lora_pkg_format"]:
            raise KeyError
        if node_cfg["lora_pkg_format"][field] is None:
            raise ValueError

    if "LORA" != node_cfg["lora_config"]["mode"]:
        raise ValueError

    if "EU868" != node_cfg["lora_config"]["region"]:
        raise ValueError

    if (863000000 > node_cfg["lora_config"]["frequency"]) or (
        870000000 < node_cfg["lora_config"]["frequency"]
    ):
        raise ValueError

    if (2 > node_cfg["lora_config"]["tx_power"]) or (
        14 < node_cfg["lora_config"]["tx_power"]
    ):
        raise ValueError

    if ("BW_125KHZ" != node_cfg["lora_config"]["bandwidth"]) and (
        "BW_250KHZ" != node_cfg["lora_config"]["bandwidth"]
    ):
        raise ValueError

    if (7 > node_cfg["lora_config"]["sf"]) or (
        12 < node_cfg["lora_config"]["sf"]
    ):
        raise ValueError

    if (
        ("CODING_4_5" != node_cfg["lora_config"]["coding_rate"])
        and ("CODING_4_6" != node_cfg["lora_config"]["coding_rate"])
        and ("CODING_4_7" != node_cfg["lora_config"]["coding_rate"])
        and ("CODING_4_8" != node_cfg["lora_config"]["coding_rate"])
    ):
        raise ValueError

    return True


if __name__ == "__main__":
    pycom.heartbeat(False)

    # * We do a lot of packing and assigning data to byte buffers, so it is
    # * prudent to proactively enable garbage collection.
    gc.enable()

    flash_led(COLOUR_RED | COLOUR_GREEN)

    try:
        with open("lib/node_cfg.json") as cfg_file:
            node_cfg = json.load(cfg_file)
    except ValueError:
        while True:
            flash_led(COLOUR_RED, times=2)
            time.sleep(10)
    except OSError:
        while True:
            flash_led(COLOUR_RED, times=4)
            time.sleep(10)
    else:
        flash_led(COLOUR_BLUE)
        if DEBUG_MODE:
            print("JSON config file loaded successfully")

    try:
        status = validate_config(node_cfg)
    except KeyError:
        while True:
            flash_led(COLOUR_RED, times=3)
            time.sleep(10)
    except ValueError:
        while True:
            flash_led(COLOUR_RED, times=5)
            time.sleep(10)
    else:
        flash_led(COLOUR_GREEN)
        if DEBUG_MODE:
            print("JSON config file validated")

    # * Moved to `struct.calcsize` rather than `len` as a better, clearer way
    # * Will keep them in the main for now
    NODE_ID_OFFSET = micropython.const(0x00)
    MSG_CNT_OFFSET = micropython.const(
        NODE_ID_OFFSET
        + struct.calcsize(node_cfg["lora_pkg_format"]["node_id"])
    )
    MSG_LEN_OFFSET = micropython.const(
        MSG_CNT_OFFSET
        + struct.calcsize(node_cfg["lora_pkg_format"]["message_cnt"])
    )
    PAYLOAD_OFFSET = micropython.const(
        MSG_LEN_OFFSET
        + struct.calcsize(node_cfg["lora_pkg_format"]["message_len"])
    )

    # ! Currently the node_id is constant throughout the duration of the
    # ! experiment, HOWEVER it could be variable in the future in case the
    # ! swarm members need reconfiguring.
    node_id = binascii.unhexlify(node_cfg["node_id"])
    node_id = int.from_bytes(node_id, "big")
    node_id = micropython.const(node_id)
    if DEBUG_MODE:
        print(node_id)

    measure_interval = node_cfg["meas_interval_sec"]
    if DEBUG_MODE:
        print(measure_interval)

    wlan_obj = WLAN(mode=WLAN.STA)
    lora_obj, lora_socket = open_lora_socket(node_cfg["lora_config"])

    # ! The radio on the node is Tx ONLY for this experiment
    lora_obj.power_mode(LoRa.TX_ONLY)

    if DEBUG_MODE:
        print("LoRa socket opened successfully")

    # * Use a `bytearray` for the LoRa packet as it is mutable and we can reuse
    # * the same buffer. It is initialised to the maximum possible value, i.e.
    # * 256 bytes, and then we can use indexing to only send what is necessary
    lora_msg_buf = bytearray(MAX_PKG_LEN)
    lora_msg_buf_ptr = memoryview(lora_msg_buf)

    flash_led(COLOUR_BLUE)

    # ! The values below are constants for this particular experiment, so are
    # ! only packed once. This might change in the future with the below being
    # ! moved to the `construct_lora_pkg` function.
    struct.pack_into(
        node_cfg["lora_pkg_format"]["node_id"],
        lora_msg_buf,
        NODE_ID_OFFSET,
        node_id,
    )

    flash_led(COLOUR_GREEN)

    experiment = machine.Timer.Alarm(
        run_experiment, measure_interval, arg=lora_msg_buf_ptr, periodic=True
    )

    # ? Does this actually work and put the ESP32 to sleep?
    machine.idle()
