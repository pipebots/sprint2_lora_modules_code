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


DEBUG_MODE = micropython.const(0x01)
COLOUR_RED = micropython.const(0xFF0000)
COLOUR_GREEN = micropython.const(0x00FFf00)
COLOUR_BLUE = micropython.const(0x0000FF)
COLOUR_OFF = micropython.const(0x000000)


def flash_led(colour=0x0, period=0.25, times=3):
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


def measure_wifi_rssi(node_cfg, wlan_obj, wlan_ssid):
    wlan_networks = wlan_obj.scan()
    for wlan_network in wlan_networks:
        if wlan_ssid in wlan_network:
            return (wlan_network.rssi, wlan_network.channel)

    return (node_cfg['meas_NA'], node_cfg['meas_NA'])


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


def construct_lora_pkg(lora_pkg_cfg, lora_msg_buf, message_count, payload):
    message_count_hex = '{:>08}'.format(hex(message_count)[2:])
    struct.pack_into(lora_pkg_cfg['message_cnt'],
                     lora_msg_buf,
                     MSG_CNT_OFFSET,
                     *(binascii.unhexlify(message_count_hex)))

    msg_len = len(payload)
    struct.pack_into(lora_pkg_cfg['message_len'],
                     lora_msg_buf,
                     MSG_LEN_OFFSET,
                     msg_len)
    struct.pack_into(lora_pkg_cfg['info_payload'],
                     lora_msg_buf,
                     PAYLOAD_OFFSET,
                     *payload)

    payload_checksum = crc32.crc32_compute(
        lora_msg_buf[:-len(lora_pkg_cfg['checksum'])]
    )
    struct.pack_into(lora_pkg_cfg['checksum'],
                     lora_msg_buf,
                     PAYLOAD_OFFSET+msg_len,
                     *payload_checksum)


def run_experiment(lora_msg_buf_ptr):
    global wlan_ssid
    global lora_socket
    global node_cfg
    global message_count

    meas_data = measure_wifi_rssi(node_cfg, wlan_obj, wlan_ssid)
    if DEBUG_MODE:
        print(meas_data)

    construct_lora_pkg(node_cfg['lora_pkg_format'], lora_msg_buf_ptr,
                       message_count, meas_data)
    if DEBUG_MODE:
        print(lora_msg_buf)

    lora_socket.send(lora_msg_buf_ptr)
    message_count += 1

    machine.idle()


pycom.heartbeat(False)
gc.enable()

flash_led(COLOUR_RED | COLOUR_GREEN)

try:
    with open('lib/node_cfg.json') as cfg_file:
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
        print('JSON config file loaded successfully')

NODE_ID_OFFSET = micropython.const(0x00)
MSG_TYPE_OFFSET = micropython.const(NODE_ID_OFFSET +
                                    len(node_cfg['lora_pkg_format']
                                                ['node_id']))
RESERVED_OFFSET = micropython.const(MSG_TYPE_OFFSET +
                                    len(node_cfg['lora_pkg_format']
                                                ['message_type']))
MSG_CNT_OFFSET = micropython.const(RESERVED_OFFSET +
                                   len(node_cfg['lora_pkg_format']
                                               ['reserved']))
MSG_LEN_OFFSET = micropython.const(MSG_CNT_OFFSET +
                                   len(node_cfg['lora_pkg_format']
                                               ['message_cnt']))
PAYLOAD_OFFSET = micropython.const(MSG_LEN_OFFSET +
                                   len(node_cfg['lora_pkg_format']
                                               ['message_len']))

node_id = micropython.const(binascii.unhexlify(node_cfg['node_id']))
if DEBUG_MODE:
    print(node_id)

message_type = micropython.const(0xAA)
reserved_value = micropython.const(0x555555)
message_count = 0x00000000
message_length = micropython.const(0x02)

measure_interval = node_cfg['meas_interval_sec']
if DEBUG_MODE:
    print(measure_interval)

wlan_ssid = node_cfg['wifi_ssid']
if DEBUG_MODE:
    print(wlan_ssid)
wlan_obj = WLAN(mode=WLAN.STA)

lora_obj, lora_socket = open_lora_socket(node_cfg['lora_config'])
lora_obj.power_mode(LoRa.TX_ONLY)
if DEBUG_MODE:
    print('LoRa socket opened successfully')
lora_pkg_length = construct_lora_pkg_format(node_cfg['lora_pkg_format'])
if DEBUG_MODE:
    print(lora_pkg_length)
lora_msg_buf = bytearray(lora_pkg_length)
lora_msg_buf_ptr = memoryview(lora_msg_buf)

flash_led(COLOUR_BLUE)

struct.pack_into(node_cfg['lora_pkg_format']['node_id'],
                 lora_msg_buf,
                 NODE_ID_OFFSET,
                 *node_id)
struct.pack_into(node_cfg['lora_pkg_format']['message_type'],
                 lora_msg_buf,
                 MSG_TYPE_OFFSET,
                 message_type)
struct.pack_into(node_cfg['lora_pkg_format']['reserved'],
                 lora_msg_buf,
                 RESERVED_OFFSET,
                 *(binascii.unhexlify(hex(reserved_value)[2:])))

flash_led(COLOUR_GREEN)

run_experiment = machine.Timer.Alarm(run_experiment, measure_interval,
                                     arg=lora_msg_buf_ptr,
                                     periodic=True)

machine.idle()
