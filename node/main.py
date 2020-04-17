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


# For diagnostic messages
DEBUG_MODE = micropython.const(0x01)

# LoPy4/FiPy use an RGB LED, where each colour is 0 - 255
COLOUR_RED = micropython.const(0xFF0000)
COLOUR_GREEN = micropython.const(0x00FFF00)
COLOUR_BLUE = micropython.const(0x0000FF)
COLOUR_OFF = micropython.const(0x000000)


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
        The socket is opened in non-blocking mode. Potentially subject to
        change in future releases.

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
    if lora_config["tx_iq"] == "True":
        tx_iq = True
    else:
        tx_iq = False

    if lora_config["rx_iq"] == "True":
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
    lora_socket.setblocking(False)

    return lora_obj, lora_socket


def measure_wifi_rssi(node_cfg, wlan_obj, wlan_ssid):
    """Measures the RSSI of a specified WLAN SSID

    This is used as a rudimentary propagation measurement experiment. We meas-
    ure the Received Signal Strength Indicator (RSSI) of a specified WLAN, as
    well as the channel that the SSID is advertised on. We do not have to be
    connected to the WLAN to take those measurements.

    Args:
        node_cfg: A `Dict` with some configuration parameters for the Pycom
                  node this is running on. Important is the 'meas_NA' value,
                  which is returned in case the specified WLAN SSID cannot
                  be detected.
        wlan_obj: A `WLAN` class giving access to the WLAN radio on-board and
                  the results from scanning for available WLANs.
        wlan_ssid: A `str` representing the WLAN SSID we are interested in.

    Returns:
        A `tuple` with the results of the measurements - the RSSI value and
        the frequency channel. If the specified SSID is not found in the scan
        results, a predefined value is returned.

    Raises:
        Nothing
    """
    wlan_networks = wlan_obj.scan()
    for wlan_network in wlan_networks:
        if wlan_ssid in wlan_network:
            return (wlan_network.rssi, wlan_network.channel)

    return (node_cfg["meas_NA"], node_cfg["meas_NA"])


def construct_lora_pkg_format(lora_pkg_cfg):
    """Constructs a representation of a RAW LoRa packet

    This function is used to determine how many bytes a particular RAW LoRa
    packet will consist of. Currently, the length in bytes of all fields is
    fixed and is specified in the JSON configuration file. The function then
    joins these together as a `struct.pack()` format string and uses
    `struct.calcsize()` to determine the overall length.

    Note:
        The way packets are constructed and handled *will* change in the
        future to support variable-length ones, and to introduce additional
        functionality.

    Args:
        lora_pkg_cfg: A `Dict` consisting of format strings indicating how
                      many bytes each field in the packet is.

    Return:
        An integer number, which currently does not change as the size of the
        payload message is fixed. This will change in the future.

    Raises:
        Nothing
    """
    header_fmt_string = "".join(
        [
            lora_pkg_cfg["node_id"],
            lora_pkg_cfg["message_type"],
            lora_pkg_cfg["reserved"],
            lora_pkg_cfg["message_cnt"],
            lora_pkg_cfg["message_len"],
            lora_pkg_cfg["info_payload"],
            lora_pkg_cfg["checksum"],
        ]
    )

    pkg_length = struct.calcsize(header_fmt_string)

    return pkg_length


def construct_lora_pkg(lora_pkg_cfg, lora_msg_buf, message_count, payload):
    """Packs experiment data in a `bytearray` buffer to be sent over LoRa link

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
        payload: *Currently* a `tuple` of two bytes. Will change in the future.

    Returns:
        The payload and the rest of the data are packed in-place in the
        `bytearray` buffer to reduce the amount of object creation and
        destruction, as well as to minimise overall memory requirements.

    Raises:
        Nothing
    """
    # * This adds padding zeroes for low values of `message_count` in order to
    # * fill four bytes worth of data.
    message_count_hex = "{:>08}".format(hex(message_count)[2:])
    struct.pack_into(
        lora_pkg_cfg["message_cnt"],
        lora_msg_buf,
        MSG_CNT_OFFSET,
        *(binascii.unhexlify(message_count_hex)),
    )

    msg_len = len(payload)
    struct.pack_into(
        lora_pkg_cfg["message_len"], lora_msg_buf, MSG_LEN_OFFSET, msg_len
    )
    struct.pack_into(
        lora_pkg_cfg["info_payload"], lora_msg_buf, PAYLOAD_OFFSET, *payload
    )

    # * The CRC32 checksum is calculated over the entire packet, including
    # * the packet counter. It is added at the end of the packet, making it
    # * reuseable in the future where the payload length will be variable.
    payload_checksum = crc32.crc32_compute(
        lora_msg_buf[: -len(lora_pkg_cfg["checksum"])]
    )
    struct.pack_into(
        lora_pkg_cfg["checksum"],
        lora_msg_buf,
        PAYLOAD_OFFSET + msg_len,
        *payload_checksum,
    )


def run_experiment(lora_msg_buf_ptr):
    """Main function of the Sprint 2 LoRa experiment - node

    This function is called every 'meas_interval_sec' number of seconds, as
    defined in the 'node_cfg' JSON file. It explicitly defines the variables
    it is going to use as global to help traceability and debugging. The body
    of the function takes a WLAN RSSI and channel measurement, packs those
    into a RAW LoRa packet, and transmits it over the air.

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
    global wlan_ssid
    global lora_socket
    global node_cfg
    global message_count

    meas_data = measure_wifi_rssi(node_cfg, wlan_obj, wlan_ssid)
    if DEBUG_MODE:
        print(meas_data)

    construct_lora_pkg(
        node_cfg["lora_pkg_format"], lora_msg_buf_ptr, message_count, meas_data
    )
    if DEBUG_MODE:
        print(lora_msg_buf)

    lora_socket.send(lora_msg_buf_ptr)
    message_count += 1

    machine.idle()


if __name__ == "__main__":
    pycom.heartbeat(False)

    # We do a lot of packing and assigning data to byte buffers, so it is
    # prudent to proactively enable garbage collection.
    gc.enable()

    flash_led(COLOUR_RED | COLOUR_GREEN)

    # TODO Add validation of JSON config file
    # TODO Handle JSON errors more gracefully rather than hanging
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

    # ? Is there a better way to construct these, or move them to a separate
    # ? function which is easier to maintain?
    NODE_ID_OFFSET = micropython.const(0x00)
    MSG_TYPE_OFFSET = micropython.const(
        NODE_ID_OFFSET + len(node_cfg["lora_pkg_format"]["node_id"])
    )
    RESERVED_OFFSET = micropython.const(
        MSG_TYPE_OFFSET + len(node_cfg["lora_pkg_format"]["message_type"])
    )
    MSG_CNT_OFFSET = micropython.const(
        RESERVED_OFFSET + len(node_cfg["lora_pkg_format"]["reserved"])
    )
    MSG_LEN_OFFSET = micropython.const(
        MSG_CNT_OFFSET + len(node_cfg["lora_pkg_format"]["message_cnt"])
    )
    PAYLOAD_OFFSET = micropython.const(
        MSG_LEN_OFFSET + len(node_cfg["lora_pkg_format"]["message_len"])
    )

    # ! Currently the node_id is constant throughout the duration of the
    # ! experiment, HOWEVER it could be variable in the future in case the
    # ! swarm members need reconfiguring.
    node_id = micropython.const(binascii.unhexlify(node_cfg["node_id"]))
    if DEBUG_MODE:
        print(node_id)

    # ! These magic values will change as the protocol is improved and its
    # ! functionality expanded
    message_type = micropython.const(0xAA)
    reserved_value = micropython.const(0x555555)
    message_length = micropython.const(0x02)

    message_count = 0x00000000

    measure_interval = node_cfg["meas_interval_sec"]
    if DEBUG_MODE:
        print(measure_interval)

    wlan_ssid = node_cfg["wifi_ssid"]
    if DEBUG_MODE:
        print(wlan_ssid)
    wlan_obj = WLAN(mode=WLAN.STA)

    lora_obj, lora_socket = open_lora_socket(node_cfg["lora_config"])

    # ! The radio is Tx ONLY just for this experiment. Will change.
    lora_obj.power_mode(LoRa.TX_ONLY)

    if DEBUG_MODE:
        print("LoRa socket opened successfully")

    lora_pkg_length = construct_lora_pkg_format(node_cfg["lora_pkg_format"])
    if DEBUG_MODE:
        print(lora_pkg_length)

    # Use a `bytearray` as it is mutable and can reuse the same buffer
    lora_msg_buf = bytearray(lora_pkg_length)
    lora_msg_buf_ptr = memoryview(lora_msg_buf)

    flash_led(COLOUR_BLUE)

    # ? Is there a better way to construct these, or move them to a separate
    # ? function which is easier to maintain?

    # * Note the unpacking of multi-byte values and the tricks to go from
    # * decimal integer to a multi-byte hexademical representation:
    # * hex(int) -> '0xABCD' -> [2:] -> 'ABCD' -> binascii.unhexlify('ABCD')
    # * -> b'\xAB\xCD'

    # ! The values below are constants for this particular experiment, so are
    # ! only packed once. This will change in the future with the below being
    # ! moved to the `construct_lora_pkg` function.
    struct.pack_into(
        node_cfg["lora_pkg_format"]["node_id"],
        lora_msg_buf,
        NODE_ID_OFFSET,
        *node_id,
    )
    struct.pack_into(
        node_cfg["lora_pkg_format"]["message_type"],
        lora_msg_buf,
        MSG_TYPE_OFFSET,
        message_type,
    )
    struct.pack_into(
        node_cfg["lora_pkg_format"]["reserved"],
        lora_msg_buf,
        RESERVED_OFFSET,
        *(binascii.unhexlify(hex(reserved_value)[2:])),
    )

    flash_led(COLOUR_GREEN)

    run_experiment = machine.Timer.Alarm(
        run_experiment, measure_interval, arg=lora_msg_buf_ptr, periodic=True
    )

    # ? Does this actually work and put the ESP32 to sleep?
    machine.idle()
