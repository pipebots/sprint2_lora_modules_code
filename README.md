```
 __________.__             ___.     |__|  __
 \______   \__|_____   ____\_ |__   _||__/  |_  ______ (C) George Jackson-Mills 2020
  |     ___/  \____ \_/ __ \| __ \ /  _ \   __\/  ___/
  |    |   |  |  |_> >  ___/| \_\ (  O_O )  |  \___ \
  |____|   |__|   __/ \___  >___  /\____/|__| /____  >
              |__|        \/    \/                 \/
```

# Sprint 2 - Minimum Viable Pipebot

## Overview
This repository contains part of the contribution of Theme 6 towards Sprint 2 of the `Pipebots` project.

In particular, this is the `MicroPython` code to be deployed on four Pycom LoPy4/FiPy boards. One of these will be integrated with the `SprintBot` developed by Theme 3. The remaining three will be colocated with the Raspberry Pi WLAN APs placed in three manholes in the ICAIR network. More information on these APs can be found in [their respective repo](https://github.com/pipebots/sprint2_rpi_wifi_network_config).

## Experiment description
The purpose of these boards is to conduct a rudimentary radio propagation experiment.

The board that is integrated with the `SprintBot` functions as a `node` which continuously measures the Received Signal Strength Indicator (RSSI) of the Raspberry Pi APs. This should give us an understanding of how well the 2.4 GHz signal propagates along the length of the sewer pipes at the ICAIR facility. For every such measurement, a `LoRa` packet is formed and transmitted using the 868 MHz radio IC.

Additionally, the boards which are colocated with the Raspberry Pis, serving as `gateways`, will be receiving these `LoRa` packets and will be recording link quality metrics for the 868 MHz channel, again an RSSI value and a Signal-to-Noise Ratio (SNR) value.

Finally, the `gateways` will transmit both the 868 MHz and 2.4 GHz data over a serial port connection to their respective Raspberry Pis for logging to a data file. Details of how this logging is done can be found in [this repo](https://github.com/pipebots/sprint2_rpi_serial_data_logger).

## Code organisation and details
The code for the `node` and the `gateway` boards is split into separate folder. The JSON configuration files will need changes before uploading to the actual boards. The `Pymakr` plugin is particularly good for that task, and is available for VS Code and other editors.

There are (hopefully) sufficient comments in the code, which can be removed prior to uploading to the boards, in order to save space.

The code has been checked against `Bandit` and has been formatted via `Black` prior to being pushed.

## Implementation notes
The code uses Pycom's version of `MicroPython` which does not contain **all** functionality. Additionally, there are some quirks as a result of the ARM architecture. Points to note are:

- ARM is `little-endian` and the implemented `int.to_bytes()` method does not support `big-endian` mode. However, both everything else is `big-endian`, which has led to an extensive use of the `binascii` and `struct` modules.
- CRC32 is not implemented, so I had to include a separate file in `lib/` which does that.
- The maximum size of a `LoRa` packet is 256 bytes, as the packet length indicator in the PHY layer is 8 bits wide.

The packet structures described in the various JSON files are subject to change as the work on custom PHY and MAC protocols progresses.

### Packet format

As mentioned above, this is preliminary and is subject to change as work progresses. However, currently, the packet format sent over the `LoRa` radio link is, with `msg` short for `message` and `cnt` short for `count`:
```
+---------+----------+----------+---------+---------+---------+-------+
| node_id | msg_type | reserved | msg_cnt | msg_len | payload | CRC32 |
+---------+----------+----------+---------+---------+---------+-------+
```
The fields are as follows:
- `node_id` - Two byte unsigned integer. Identifies the transmitter.
- `msg_type` - One byte unsigned integer. Currently has a fixed magical value of `0xAA`. In the future can be used to differentiate where the `payload` originated from, whether the `payload` is useful data, telemetry, or link control.
- `reserved` - Three unsigned bytes. In case we have missed something and need to add more functionality.
- `msg_cnt` - Four byte unsigned integer. Will be used to keep track of lost packets and request re-transmission.
- `msg_len` - One byte unsigned integer. The length in bytes of the `payload` field.
- `payload` - **_Currently_** two signed bytes, one for the WLAN RSSI and one for the WLAN channel. **_Will_** change to support more data, up to 241 bytes. The reason for this seemingly random number is that the `LoRa` PHY packet accepts a payload of no more than 256 bytes.
- `CRC32` - Four byte unsigned integer. A CRC32 checksum.

## TO DO
- More checks and validations of various inputs and loaded files such as:
  - Whether all fields are present
  - Whether the values for those are valid
  - Type checking of inputs
- Better error handling - potentially load a default configuration and issue an error message rather than hang up in an infinite loop
- Improve functionality to allow a variable-length payload
- Create an API to enable transparent use of the LoPy4/FiPy boards by other Themes
- Add unit and integration testing - difficult due to need for extensive mocking
- Add CI for `Black` and `Bandit`

## Contributing

Contributions are more than welcome and are in fact actively sought! Please contact Viktor either at [eenvdo@leeds.ac.uk](mailto:eenvdo@leeds.ac.uk) or on the `Pipebots` Slack.

Particularly looking for feedback and requirements on types of communication each Theme would require, i.e. broadcast/multicast/point-to-point, frequency, expected payload size, and so on.