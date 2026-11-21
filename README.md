Just a very quick library for Espressif's IDE/IDF for a CC1101 tranceiver.

## Two-radio ESP32 test console

[`examples/two_radio_test`](examples/two_radio_test) is an ESP-IDF project for an original
ESP32 DevKit and two 433 MHz CC1101 modules. It checks transfers in both directions and
compares each received payload; it is not a substitute for RF conformance testing.

| CC1101 pin | Radio A | Radio B |
| --- | --- | --- |
| VCC | ESP32 3.3 V | ESP32 3.3 V |
| GND | ESP32 GND | ESP32 GND |
| SCK | GPIO18 | GPIO18 |
| SO / MISO | GPIO19 | GPIO19 |
| SI / MOSI | GPIO23 | GPIO23 |
| CSN | GPIO25 | GPIO26 |

Use 3.3 V only, a common ground, and suitable 433 MHz antennas. Do not connect radio
outputs directly together. Pull each CSN up to 3.3 V (for example 10 kOhm) so both
radios remain deselected during ESP32 reset. GDO pins are not needed. Keep the
antennas separated; very close radios can overload the receiver. GPIO assignments
and frequency are constants in the example and can be changed for your board.
Both radios use 433.92 MHz, GFSK, 4.8 kbaud and the library's default packet format.
Ensure the frequency and transmissions are legal in your region.

With ESP-IDF 5.x installed, open the example folder as an ESP-IDF project or run:

```sh
cd examples/two_radio_test
idf.py set-target esp32
idf.py build flash monitor
```

In the serial monitor (default 115200 baud), send lines using Enter:

* `status` prints chip identity and state.
* `send a hello` tests A to B; `send b hello` tests B to A.
* `test` runs five packets in each direction; `test 20` runs 20 each (maximum 100).
* `help` lists commands.

Each exchange reports pass/fail, RSSI and LQI on success. A receive timeout, CRC
error, or unexpected payload is a failure, not a successful transmit. On startup,
an initialization failure stops the console; check wiring and power and reset the ESP32.
For timeouts, check antennas, separation, power, and that both modules match the
selected frequency. This test requires both modules; it does not automatically test
every driver configuration or report unsolicited received packets.
