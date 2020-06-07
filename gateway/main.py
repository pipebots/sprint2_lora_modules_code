"""LoPy4/FiPy LoRa gateway firmware for Pipebots Sprint 2

This is the MicroPython firmware for the LoPy4/FiPy gateways that will be
placed at three strategic manholes. These are responsible for receiving the
LoRa packets sent by the SprintBot LoRa node, unpacking the data, adding info
on the LoRa link quality, and transmitting everything over a serial port to
a Raspberry Pi which will log the data to an SD card.

This file needs to be uploaded/flashed to a LoPy4/FiPy board together with the
lib/crc32.py and lib/gateway_cfg.json files. The latter one needs customising
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

from machine import UART
from network import WLAN
from network import LoRa

from lib import crc32


# * For diagnostic messages
DEBUG_MODE = micropython.const(0x01)
USB_SERIAL = micropython.const(0x00)

# * LoPy4/FiPy use an RGB LED, where each colour is 0 - 255
COLOUR_RED = micropython.const(0xFF0000)
COLOUR_GREEN = micropython.const(0x00FFF00)
COLOUR_BLUE = micropython.const(0x0000FF)
COLOUR_OFF = micropython.const(0x000000)

MAX_PKG_LEN = micropython.const(0x100)


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
    lora_socket.setblocking(False)

    return (lora_obj, lora_socket)


def sync_ntp_time(ntp_sync_cfg):
    """Syncs the LoPy4/FiPy RTC with an NTP server

    The LoPy4/FiPy set their internal clocks to the start of the UNIX epoch
    upon bootup. We want to synchronise them to an NTP server to enable time-
    stamping of incoming packets. The gateways connect to the Sprint 2 WLAN
    APs just to perform this sync, and then turn off their WLAN radios.

    Args:
        ntp_sync_cfg: A `Dict` with configuration settings, such as AP SSID,
                      password, and NTP server URL.

    Returns:
        None

    Raises:
        Nothing
    """

    wlan_obj = WLAN(mode=WLAN.STA)
    wlan_obj.connect(
        ssid=ntp_sync_cfg["wifi_ssid"],
        auth=(WLAN.WPA2, ntp_sync_cfg["wifi_password"]),
    )
    while not wlan_obj.isconnected():
        time.sleep(0.05)

    if DEBUG_MODE:
        print("Wi-Fi connection established")

    # * I assume this request happens in the background in a separate thread
    # * as there is no option to stop attempting to sync after a few
    # * unsuccessful attempts.
    rtc_obj = machine.RTC()
    rtc_obj.ntp_sync(server=ntp_sync_cfg["ntp_server"])
    while not rtc_obj.synced():
        time.sleep(1)

    if DEBUG_MODE:
        print("Time synced")

    # * This is required to stop the periodic re-syncing, as we immediately
    # * disconnect from and disable the WLAN radio.
    rtc_obj.ntp_sync(server=None)

    wlan_obj.disconnect()
    wlan_obj.deinit()


def setup_serial_port(serial_port_cfg):
    """Sets up and opens an UART port to a host computer

    Currently an UART connection is the best way for the LoPy4/FiPy gateways
    to offload data to a computer. The setup is straightforward, with some
    helper dictionaries used to convert the strings from a JSON file to the
    built-in numeric constants.

    Note:
        For a more detailed description of what port parameters are and which
        are needed, including restrictions on values, have a look at the Pycom
        website - https://docs.pycom.io/firmwareapi/pycom/machine/uart/

    Args:
        serial_port_cfg: A `Dict` with the port parameters

    Returns:
        An `UART` object which can be used to send and receive data by other
        functions. Can also be used to alter the parameters of the port though
        in reality that is unlikely to happen.

    Raises:
        Nothing
    """

    bytesize = {"FIVEBITS": 5, "SIXBITS": 6, "SEVENBITS": 7, "EIGHTBITS": 8}

    parity = {
        "PARITY_NONE": None,
        "PARITY_EVEN": UART.EVEN,
        "PARITY_ODD": UART.ODD,
    }

    stopbits = {"STOPBITS_ONE": 1, "STOPBITS_TWO": 2}

    # ! This is specific to the LoPy4/FiPy, and is the defaul serial timeout
    if "none" == serial_port_cfg["timeout"].lower():
        serial_port_cfg["timeout"] = 2

    serial_port = UART(
        serial_port_cfg["port"],
        baudrate=serial_port_cfg["baudrate"],
        bits=bytesize[serial_port_cfg["bytesize"]],
        parity=parity[serial_port_cfg["parity"]],
        stop=stopbits[serial_port_cfg["stopbits"]],
        timeout_chars=serial_port_cfg["timeout"],
        pins=(serial_port_cfg["tx_pin"], serial_port_cfg["rx_pin"]),
    )

    return serial_port


def decode_lora_pkg(lora_pkg_cfg, payload_contents, lora_msg_buf_ptr):
    """Unpacks a LoRa packet into a dictionary

    The RAW LoRa packet is a sequence of `bytes` when received and needs to be
    unpacked, with multi-byte variables needing reconstruction.

    Note:
        This function has been updated to work with a variable-length payload,
        however CRC32 validation is still not performed. This will be added
        when we transition to a proper data link layer protocol.

    Args:
        lora_pkg_cfg: A `Dict` describing the structure of a RAW LoRa packet
        payload_contents: A `Dict` describing the payload contents
        lora_msg_buf_ptr: A `bytearray` or a `memoryview` into one, which
                          should contain a valid RAW LoRa packet as described
                          by `lora_pkg_cfg`.

    Returns:
        A dictionary containing the values for each of the RAW LoRa packet
        fields. Multi-byte ones are combined using the `from_bytes()` method.

    Raises:
        RuntimeError: If a malformed packet has been received and the payload
                      length is not one of the predefined sizes and formats.
    """

    global OFFSETS

    lora_rx_data = {}
    payload_data = {}

    # * We need to find out how many bytes of data we have before we have the
    # * CRC32 checksum bytes. Unlike the previous version, the `checksum`
    # * offset is variable and needs to be recalculated every packet.
    payload_size = struct.unpack_from(
        lora_pkg_cfg["message_len"], lora_msg_buf_ptr, MSG_LEN_OFFSET
    )
    payload_size = payload_size[0]
    if DEBUG_MODE:
        print(payload_size)

    OFFSETS["checksum"] = PAYLOAD_OFFSET + payload_size

    # * The use of `8` and `3` as magic values is poor practice, however
    # * this release has already taken far too long, and future plans will
    # * not build on it anyway. Ideally there would be another function that
    # * calculates the size for the different configurations of the payload.
    if 8 == payload_size:
        meas_state = "valid"
    elif 3 == payload_size:
        meas_state = "invalid"
    else:
        if DEBUG_MODE:
            print(payload_size)
        raise RuntimeError

    # ! The payload is three values of different bit/byte length, so
    # ! they should not be combined together. The payload length also
    # ! varies depending on whether the WLAN to be measured had been
    # ! in range or not.
    for element in lora_pkg_cfg:
        if "info_payload" == element:
            payload_field_offset = OFFSETS[element]
            for payload_element in payload_contents["order"]:
                payload_data[payload_element] = struct.unpack_from(
                    payload_contents[meas_state][payload_element],
                    lora_msg_buf_ptr,
                    payload_field_offset,
                )
                payload_field_offset += struct.calcsize(
                    payload_contents[meas_state][payload_element]
                )
        else:
            lora_rx_data[element] = struct.unpack_from(
                lora_pkg_cfg[element], lora_msg_buf_ptr, OFFSETS[element]
            )
            lora_rx_data[element] = lora_rx_data[element][0]

    lora_rx_data["info_payload"] = payload_data

    return lora_rx_data


def construct_log_string(gateway_cfg, lora_rx_data, lora_stats):
    """Combines data from LoRa with other diagnostics for logging to a computer

    The data received in a LoRa packet is converted to strings and is combined
    with other metrics, such as observed LoRa RSSI and SNR at the receiver.
    Additionally, the gateway timestamp is formatted and added in an
    YYYY-MM-DD HH:MM:SS format. Everything is joined together in a single
    string in order specified in the JSON config file.

    Note:
        Currently the different fields are appended to a list manually, this
        should change in the future for a less error-prone approach.

    Args:
        gateway_cfg: A `Dict` with the gateway configuration. The fields that
                     are actually used are the `gateway_id` and the
                     `serial_pkg_format` one, which specifies the order in
                     which to combine the individual fields.
        lora_rx_data: A `Dict` with the unpacked data from the most recently
                      received LoRa packet.
        lora_stats: A `NamedTuple` with the LoRa metrics for the most recently
                    received LoRa packet.

    Returns:
        A string terminated with a newline character, consisting of all the
        fields specified in the `serial_pkg_format` and their corresponding
        values.
    """

    serial_tx_data = []

    serial_tx_data.append(gateway_cfg["gateway_id"])

    gateway_time = time.gmtime()
    gateway_time = "{}-{}-{} {}:{}:{}".format(*gateway_time[:6])
    serial_tx_data.append(gateway_time)

    lora_rx_time = lora_stats.rx_timestamp
    if DEBUG_MODE:
        print(lora_rx_time)
    lora_rx_time = str(lora_rx_time)
    serial_tx_data.append(lora_rx_time)

    lora_rssi = str(lora_stats.rssi)
    serial_tx_data.append(lora_rssi)

    lora_snr = str(lora_stats.snr)
    serial_tx_data.append(lora_snr)

    wlan_rssi = str(lora_rx_data["info_payload"]["wlan_rssi"])
    serial_tx_data.append(wlan_rssi)

    wlan_chan = str(lora_rx_data["info_payload"]["wlan_channel"])
    serial_tx_data.append(wlan_chan)

    wlan_bssid = str(lora_rx_data["info_payload"]["wlan_bssid"])
    serial_tx_data.append(wlan_bssid)

    for element in gateway_cfg["serial_pkg_format"]:
        if element in lora_rx_data and element != "info_payload":
            serial_tx_data.append(str(lora_rx_data[element]))

    log_string = []

    for field_header, field_data in zip(
        gateway_cfg["serial_pkg_format"], serial_tx_data
    ):
        log_string.append(field_header)
        log_string.append(field_data)

    # * The separator could be turned into a configured parameter too
    # * More importantly, need to manually terminate the line with a '\n'
    log_string = ",".join(log_string)
    log_string = "".join([log_string, "\n"])

    return log_string


def on_lora_rx_packet(lora_msg_buf_ptr):
    """Main function of the Sprint 2 LoRa experiment - gateway

    This function is called whenever the LoRa radio indicates a packet has
    been received. The packet is unpacked and the data extracted, combined
    with additional metrics, converted to a string, encoded, and then sent
    over an UART connection a computer for further processing and logging.

    Note:
        The callback mechanism allows for a single argument to be passed to
        the function, which is why the `global` definitions are needed.

    Args:
        lora_msg_buf_ptr: A `memoryview` into a `bytearray` object, which is
                          the received data.

    Returns:
        None

    Raises:
        Nothing
    """

    global lora_obj
    global lora_socket
    global gateway_cfg

    lora_msg_buf_ptr = lora_socket.recv(MAX_PKG_LEN)
    lora_stats = lora_obj.stats()

    # * Visual feedback that the node is doing something
    flash_led(COLOUR_BLUE, 0.1, 1)

    try:
        lora_rx_data = decode_lora_pkg(
            gateway_cfg["lora_pkg_format"],
            gateway_cfg["payload_contents"],
            lora_msg_buf_ptr,
        )
    except RuntimeError:
        flash_led(COLOUR_RED, 0.1, 1)
        return None

    if DEBUG_MODE:
        print(lora_rx_data)

    serial_tx_data = construct_log_string(
        gateway_cfg, lora_rx_data, lora_stats
    )
    if USB_SERIAL:
        print(serial_tx_data)

    serial_tx_data = serial_tx_data.encode()
    serial_port.write(serial_tx_data)
    if DEBUG_MODE:
        print(serial_tx_data)

    flash_led(COLOUR_GREEN, 0.1, 1)
    machine.idle()


def validate_config(gateway_cfg):
    """Perform validation of JSON config file

    This does a quick check if all necessary fields were present in the JSON
    configuration file. There is also a series of checks if the values for the
    LoRa configuration are correct. Perhaps overkill, but better safe than
    sorry.

    Args:
        gateway_cfg: A `Dict` with the configuration settings for the LoRa node

    Returns:
        True if all checks pass

    Raises:
        KeyError: If at least one configuration parameter was not present
        ValueError: If there is an incorrect value for the LoRa configuration,
                    or if any of the other parameters were empty/null.
    """

    _MAIN_FIELDS = [
        "gateway_id",
        "ntp_sync_cfg",
        "serial_port_config",
        "serial_pkg_format",
        "lora_config",
        "lora_pkg_format",
        "payload_contents",
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

    _SERIAL_PORT_FIELDS = [
        "port",
        "baudrate",
        "bytesize",
        "parity",
        "stopbits",
        "timeout",
        "tx_pin",
        "rx_pin",
    ]

    _NTP_FIELDS = ["wifi_ssid", "wifi_password", "ntp_server"]

    for field in _MAIN_FIELDS:
        if field not in gateway_cfg:
            raise KeyError
        if gateway_cfg[field] is None:
            raise ValueError

    for field in _LORA_FIELDS:
        if field not in gateway_cfg["lora_config"]:
            raise KeyError
        if gateway_cfg["lora_config"][field] is None:
            raise ValueError

    for field in _LORA_PKG_FIELDS:
        if field not in gateway_cfg["lora_pkg_format"]:
            raise KeyError
        if gateway_cfg["lora_pkg_format"][field] is None:
            raise ValueError

    for field in _SERIAL_PORT_FIELDS:
        if field not in gateway_cfg["serial_port_config"]:
            raise KeyError
        if gateway_cfg["serial_port_config"][field] is None:
            raise ValueError

    for field in _NTP_FIELDS:
        if field not in gateway_cfg["ntp_sync_cfg"]:
            raise KeyError
        if gateway_cfg["ntp_sync_cfg"][field] is None:
            raise ValueError

    if "LORA" != gateway_cfg["lora_config"]["mode"]:
        raise ValueError

    if "EU868" != gateway_cfg["lora_config"]["region"]:
        raise ValueError

    if (863000000 > gateway_cfg["lora_config"]["frequency"]) or (
        870000000 < gateway_cfg["lora_config"]["frequency"]
    ):
        raise ValueError

    if (2 > gateway_cfg["lora_config"]["tx_power"]) or (
        14 < gateway_cfg["lora_config"]["tx_power"]
    ):
        raise ValueError

    if ("BW_125KHZ" != gateway_cfg["lora_config"]["bandwidth"]) and (
        "BW_250KHZ" != gateway_cfg["lora_config"]["bandwidth"]
    ):
        raise ValueError

    if (7 > gateway_cfg["lora_config"]["sf"]) or (
        12 < gateway_cfg["lora_config"]["sf"]
    ):
        raise ValueError

    if (
        ("CODING_4_5" != gateway_cfg["lora_config"]["coding_rate"])
        and ("CODING_4_6" != gateway_cfg["lora_config"]["coding_rate"])
        and ("CODING_4_7" != gateway_cfg["lora_config"]["coding_rate"])
        and ("CODING_4_8" != gateway_cfg["lora_config"]["coding_rate"])
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
        with open("lib/gateway_cfg.json") as cfg_file:
            gateway_cfg = json.load(cfg_file)
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
        status = validate_config(gateway_cfg)
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
        + struct.calcsize(gateway_cfg["lora_pkg_format"]["node_id"])
    )
    MSG_LEN_OFFSET = micropython.const(
        MSG_CNT_OFFSET
        + struct.calcsize(gateway_cfg["lora_pkg_format"]["message_cnt"])
    )
    PAYLOAD_OFFSET = micropython.const(
        MSG_LEN_OFFSET
        + struct.calcsize(gateway_cfg["lora_pkg_format"]["message_len"])
    )

    OFFSETS = {
        "node_id": NODE_ID_OFFSET,
        "message_cnt": MSG_CNT_OFFSET,
        "message_len": MSG_LEN_OFFSET,
        "info_payload": PAYLOAD_OFFSET,
    }

    sync_ntp_time(gateway_cfg["ntp_sync_cfg"])

    serial_port = setup_serial_port(gateway_cfg["serial_port_config"])
    if DEBUG_MODE:
        print("UART port opened successfully")

    flash_led(COLOUR_BLUE)

    lora_obj, lora_socket = open_lora_socket(gateway_cfg["lora_config"])

    # ! The radio is ALWAYS_ON unlike the one in the node. Likely to stay like
    # ! this as it is assumed the gateway will be connected to mains power.
    lora_obj.power_mode(LoRa.ALWAYS_ON)
    if DEBUG_MODE:
        print("LoRa socket opened successfully")

    flash_led(COLOUR_BLUE)

    # * Use a `bytearray` as it is mutable and can reuse the same buffer
    lora_msg_buf = bytearray(MAX_PKG_LEN)
    lora_msg_buf_ptr = memoryview(lora_msg_buf)

    flash_led(COLOUR_GREEN)

    lora_obj.callback(
        trigger=LoRa.RX_PACKET_EVENT,
        handler=on_lora_rx_packet,
        arg=lora_msg_buf_ptr,
    )

    machine.idle()
