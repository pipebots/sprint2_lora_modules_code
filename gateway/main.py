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
    if "True" == lora_config["tx_iq"]:
        tx_iq = True
    else:
        tx_iq = False

    if "True" == lora_config["rx_iq"]:
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

    if "None" == serial_port_cfg["timeout"]:
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


def decode_lora_pkg(lora_pkg_cfg, lora_msg_buf_ptr):
    """Unpacks a LoRa packet into a dictionary

    The RAW LoRa packet is a sequence of `bytes` when received and needs to be
    unpacked, with multi-byte variables needing reconstruction.

    Note:
        Currently this function assumes a fixed payload and does not perform
        any length checks, nor CRC32 validation. This will change in the
        future releases.

    Args:
        lora_pkg_cfg: A `Dict` describing the structure of a RAW LoRa packet
        lora_msg_buf_ptr: A `bytearray` or a `memoryview` into one, which
                          should contain a valid RAW LoRa packet as described
                          by 'lora_pkg_cfg'.

    Returns:
        A dictionary containing the values for each of the RAW LoRa packet
        fields. Multi-byte ones are combined using the `from_bytes()` method.

    Raises:
        Nothing
    """
    lora_rx_data = {}

    for element in lora_pkg_cfg:
        lora_rx_data[element] = struct.unpack_from(
            lora_pkg_cfg[element], lora_msg_buf_ptr, OFFSETS[element]
        )

        # ! The payload is two *separate* bytes for different things,
        # ! therefore it should not be combined into a single number.
        # ! This will change in the future as we move to variable-
        # ! length payloads.
        if element != "info_payload":
            lora_rx_data[element] = int.from_bytes(
                bytes(lora_rx_data[element]), "big"
            )

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
                     are actually used are the 'gateway_id' and the
                     'serial_pkg_format' one, which specifies the order in
                     which to combine the individual fields.
        lora_rx_data: A `Dict` with the unpacked data from the most recently
                      received LoRa packet.
        lora_stats: A `NamedTuple` with the LoRa metrics for the most recently
                    received LoRa packet.

    Returns:
        A string terminated with a newline character, consisting of all the
        fields specified in the 'serial_pkg_format' and their corresponding
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

    wifi_rssi = str(lora_rx_data["info_payload"][0])
    serial_tx_data.append(wifi_rssi)

    wifi_chan = str(lora_rx_data["info_payload"][1])
    serial_tx_data.append(wifi_chan)

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
    global lora_pkg_length
    global gateway_cfg

    lora_msg_buf_ptr = lora_socket.recv(lora_pkg_length)
    lora_stats = lora_obj.stats()

    lora_rx_data = decode_lora_pkg(
        gateway_cfg["lora_pkg_format"], lora_msg_buf_ptr
    )
    if DEBUG_MODE:
        print(lora_rx_data)

    serial_tx_data = construct_log_string(
        gateway_cfg, lora_rx_data, lora_stats
    )
    serial_tx_data = serial_tx_data.encode()

    serial_port.write(serial_tx_data)
    if DEBUG_MODE:
        print(serial_tx_data)

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

    # ? Is there a better way to construct these, or move them to a separate
    # ? function which is easier to maintain?
    NODE_ID_OFFSET = micropython.const(0x00)
    MSG_TYPE_OFFSET = micropython.const(
        NODE_ID_OFFSET + len(gateway_cfg["lora_pkg_format"]["node_id"])
    )
    RESERVED_OFFSET = micropython.const(
        MSG_TYPE_OFFSET + len(gateway_cfg["lora_pkg_format"]["message_type"])
    )
    MSG_CNT_OFFSET = micropython.const(
        RESERVED_OFFSET + len(gateway_cfg["lora_pkg_format"]["reserved"])
    )
    MSG_LEN_OFFSET = micropython.const(
        MSG_CNT_OFFSET + len(gateway_cfg["lora_pkg_format"]["message_cnt"])
    )
    PAYLOAD_OFFSET = micropython.const(
        MSG_LEN_OFFSET + len(gateway_cfg["lora_pkg_format"]["message_len"])
    )

    OFFSETS = {
        "node_id": NODE_ID_OFFSET,
        "message_type": MSG_TYPE_OFFSET,
        "reserved": RESERVED_OFFSET,
        "message_cnt": MSG_CNT_OFFSET,
        "message_len": MSG_LEN_OFFSET,
        "info_payload": PAYLOAD_OFFSET,
    }

    sync_ntp_time(gateway_cfg["ntp_sync_cfg"])

    lora_obj, lora_socket = open_lora_socket(gateway_cfg["lora_config"])

    # ! The radio is ALWAYS_ON unlike the one in the node. Likely to stay like
    # ! this as it is assumed the gateway will be connected to mains power.
    lora_obj.power_mode(LoRa.ALWAYS_ON)
    if DEBUG_MODE:
        print("LoRa socket opened successfully")

    flash_led(COLOUR_BLUE)

    serial_port = setup_serial_port(gateway_cfg["serial_port_config"])
    if DEBUG_MODE:
        print("UART port opened successfully")

    flash_led(COLOUR_BLUE)

    lora_pkg_length = construct_lora_pkg_format(gateway_cfg["lora_pkg_format"])
    if DEBUG_MODE:
        print(lora_pkg_length)

    CHECKSUM_OFFSET = micropython.const(
        lora_pkg_length - len(gateway_cfg["lora_pkg_format"]["checksum"])
    )
    OFFSETS["checksum"] = CHECKSUM_OFFSET

    # Use a `bytearray` as it is mutable and can reuse the same buffer
    lora_msg_buf = bytearray(lora_pkg_length)
    lora_msg_buf_ptr = memoryview(lora_msg_buf)

    flash_led(COLOUR_GREEN)

    lora_obj.callback(
        trigger=LoRa.RX_PACKET_EVENT,
        handler=on_lora_rx_packet,
        arg=lora_msg_buf_ptr,
    )

    machine.idle()
