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


DEBUG_MODE = micropython.const(1)
COLOUR_RED = micropython.const(0xFF0000)
COLOUR_GREEN = micropython.const(0x00FFf00)
COLOUR_BLUE = micropython.const(0x0000FF)
COLOUR_OFF = micropython.const(0x000000)


def flash_led(colour=COLOUR_OFF, period=0.25, times=3):
    for _ in range(times):
        pycom.rgbled(colour)
        time.sleep(period)
        pycom.rgbled(COLOUR_OFF)
        time.sleep(period)

    pycom.rgbled(COLOUR_OFF)


def open_lora_socket(lora_config):
    if lora_config['tx_iq'] == 'True':
        tx_iq = True
    else:
        tx_iq = False

    if lora_config['rx_iq'] == 'True':
        rx_iq = True
    else:
        rx_iq = False

    lora_obj = LoRa(mode=getattr(LoRa, lora_config['mode']),
                    region=getattr(LoRa, lora_config['region']),
                    bandwidth=getattr(LoRa, lora_config['bandwidth']),
                    coding_rate=getattr(LoRa, lora_config['coding_rate']),
                    frequency=lora_config['frequency'],
                    tx_power=lora_config['tx_power'],
                    sf=lora_config['sf'],
                    tx_iq=tx_iq,
                    rx_iq=rx_iq)

    lora_socket = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
    lora_socket.setblocking(False)

    return lora_obj, lora_socket


def sync_ntp_time(ntp_sync_cfg):
    wlan_obj = WLAN(mode=WLAN.STA)
    wlan_obj.connect(ssid=ntp_sync_cfg['wifi_ssid'],
                     auth=(WLAN.WPA2, ntp_sync_cfg['wifi_password']))
    while not wlan_obj.isconnected():
        time.sleep(0.05)

    if DEBUG_MODE:
        print('Wi-Fi connection established')

    rtc_obj = machine.RTC()
    rtc_obj.ntp_sync(server=ntp_sync_cfg['ntp_server'])
    while not rtc_obj.synced():
        time.sleep(1)

    if DEBUG_MODE:
        print('Time synced')

    rtc_obj.ntp_sync(server=None)

    wlan_obj.disconnect()
    wlan_obj.deinit()


def setup_serial_port(serial_port_cfg):
    bytesize = {'FIVEBITS': 5, 'SIXBITS': 6, 'SEVENBITS': 7, 'EIGHTBITS': 8}
    parity = {'PARITY_NONE': None, 'PARITY_EVEN': UART.EVEN,
              'PARITY_ODD': UART.ODD}
    stopbits = {'STOPBITS_ONE': 1, 'STOPBITS_TWO': 2}
    if serial_port_cfg['timeout'] == 'None':
        serial_port_cfg['timeout'] = 2

    serial_port = UART(serial_port_cfg['port'],
                       baudrate=serial_port_cfg['baudrate'],
                       bits=bytesize[serial_port_cfg['bytesize']],
                       parity=parity[serial_port_cfg['parity']],
                       stop=stopbits[serial_port_cfg['stopbits']],
                       timeout_chars=serial_port_cfg['timeout'],
                       pins=(serial_port_cfg['tx_pin'],
                             serial_port_cfg['rx_pin']))

    return serial_port


def construct_lora_pkg_format(lora_pkg_cfg):
    header_fmt_string = ''.join([lora_pkg_cfg['node_id'],
                                 lora_pkg_cfg['message_type'],
                                 lora_pkg_cfg['reserved'],
                                 lora_pkg_cfg['message_cnt'],
                                 lora_pkg_cfg['message_len'],
                                 lora_pkg_cfg['info_payload'],
                                 lora_pkg_cfg['checksum']])

    pkg_length = struct.calcsize(header_fmt_string)

    return pkg_length


def decode_lora_pkg(lora_pkg_cfg, lora_msg_buf_ptr):
    lora_rx_data = {}

    for element in lora_pkg_cfg:
        lora_rx_data[element] = struct.unpack_from(lora_pkg_cfg[element],
                                                   lora_msg_buf_ptr,
                                                   OFFSETS[element])
        if element != 'info_payload':
            lora_rx_data[element] = int.from_bytes(
                bytes(lora_rx_data[element]), 'big')

    return lora_rx_data


def construct_log_string(gateway_cfg, lora_rx_data, lora_stats):
    serial_tx_data = []

    serial_tx_data.append(gateway_cfg['gateway_id'])

    gateway_time = time.gmtime()
    gateway_time = '{}-{}-{} {}:{}:{}'.format(*gateway_time[:6])
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

    wifi_rssi = str(lora_rx_data['info_payload'][0])
    serial_tx_data.append(wifi_rssi)

    wifi_chan = str(lora_rx_data['info_payload'][1])
    serial_tx_data.append(wifi_chan)

    for element in gateway_cfg['serial_pkg_format']:
        if element in lora_rx_data and element != 'info_payload':
            serial_tx_data.append(str(lora_rx_data[element]))

    log_string = []

    for field_header, field_data in zip(gateway_cfg['serial_pkg_format'],
                                        serial_tx_data):
        log_string.append(field_header)
        log_string.append(field_data)

    log_string = ','.join(log_string)
    log_string = ''.join([log_string, '\n'])

    return log_string


def on_lora_rx_packet(lora_msg_buf_ptr):
    global lora_obj
    global lora_socket
    global lora_pkg_length
    global gateway_cfg

    lora_msg_buf_ptr = lora_socket.recv(lora_pkg_length)
    lora_stats = lora_obj.stats()

    lora_rx_data = decode_lora_pkg(gateway_cfg['lora_pkg_format'],
                                   lora_msg_buf_ptr)
    if DEBUG_MODE:
        print(lora_rx_data)

    serial_tx_data = construct_log_string(gateway_cfg,
                                          lora_rx_data, lora_stats)
    serial_tx_data = serial_tx_data.encode()

    serial_port.write(serial_tx_data)
    if DEBUG_MODE:
        print(serial_tx_data)

    machine.idle()


pycom.heartbeat(False)
gc.enable()

flash_led(COLOUR_RED | COLOUR_GREEN)

try:
    with open('lib/gateway_cfg.json') as cfg_file:
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
        print('JSON config file loaded successfully')

NODE_ID_OFFSET = micropython.const(0x00)
MSG_TYPE_OFFSET = micropython.const(NODE_ID_OFFSET +
                                    len(gateway_cfg['lora_pkg_format']
                                                   ['node_id']))
RESERVED_OFFSET = micropython.const(MSG_TYPE_OFFSET +
                                    len(gateway_cfg['lora_pkg_format']
                                                   ['message_type']))
MSG_CNT_OFFSET = micropython.const(RESERVED_OFFSET +
                                   len(gateway_cfg['lora_pkg_format']
                                                  ['reserved']))
MSG_LEN_OFFSET = micropython.const(MSG_CNT_OFFSET +
                                   len(gateway_cfg['lora_pkg_format']
                                                  ['message_cnt']))
PAYLOAD_OFFSET = micropython.const(MSG_LEN_OFFSET +
                                   len(gateway_cfg['lora_pkg_format']
                                                  ['message_len']))

OFFSETS = {'node_id': NODE_ID_OFFSET, 'message_type': MSG_TYPE_OFFSET,
           'reserved': RESERVED_OFFSET, 'message_cnt': MSG_CNT_OFFSET,
           'message_len': MSG_LEN_OFFSET, 'info_payload': PAYLOAD_OFFSET}

sync_ntp_time(gateway_cfg['ntp_sync_cfg'])

lora_obj, lora_socket = open_lora_socket(gateway_cfg['lora_config'])
lora_obj.power_mode(LoRa.ALWAYS_ON)
if DEBUG_MODE:
    print('LoRa socket opened successfully')

flash_led(COLOUR_BLUE)

serial_port = setup_serial_port(gateway_cfg['serial_port_config'])
if DEBUG_MODE:
    print('UART port opened successfully')

flash_led(COLOUR_BLUE)

lora_pkg_length = construct_lora_pkg_format(gateway_cfg['lora_pkg_format'])
if DEBUG_MODE:
    print(lora_pkg_length)

CHECKSUM_OFFSET = micropython.const(lora_pkg_length -
                                    len(gateway_cfg['lora_pkg_format']
                                                   ['checksum']))
OFFSETS['checksum'] = CHECKSUM_OFFSET

lora_msg_buf = bytearray(lora_pkg_length)
lora_msg_buf_ptr = memoryview(lora_msg_buf)

flash_led(COLOUR_GREEN)

lora_obj.callback(trigger=LoRa.RX_PACKET_EVENT, handler=on_lora_rx_packet,
                  arg=lora_msg_buf_ptr)

machine.idle()
