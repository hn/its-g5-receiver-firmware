# ITS-G5 Receiver Firmware

## Flashing a pre-compiled firmware

If you can't or don't want to build the firmware yourself, you can download prebuilt files.

You need the following files:

- [`bootloader_0.1.0.bin`](https://codeberg.org/opentrafficmap/its-g5-receiver-firmware/releases/download/0.1.0/bootloader_0.1.0.bin)
- [`partition-table_8M_0.1.0.bin`](https://codeberg.org/opentrafficmap/its-g5-receiver-firmware/releases/download/0.1.0/partition-table_8M_0.1.0.bin)
- [latest firmware file](https://codeberg.org/opentrafficmap/its-g5-receiver-firmware/releases) ending in `.bin`

To flash them to a board you need `esptool` which is included with the ESP-IDF.

On macOS you can install `esptool` via [Homebrew](https://brew.sh/).

```sh
brew install esptool
```

Make sure to replace the device name of the serial port with the one where the ESP is connected.
Substitute the firmware file name for the one of the latest release that you downloaded.

You can flash them to your board with a command like the following:

```sh
esptool --chip esp32c5 -p /dev/ttyACM0 -b 921600 --before=default-reset --after=hard-reset write-flash --flash-mode dio --flash-freq 80m --flash-size 8MB 0x2000 bootloader_0.1.0.bin 0x8000 partition-table_8M_0.1.0.bin 0x20000 its-g5-receiver-firmware_i5r-r1_0.5.1.bin
```

## Cloning

```sh
git clone https://codeberg.org/opentrafficmap/its-g5-receiver-firmware.git
cd its-g5-receiver-firmware
git submodule update --init --recursive
```

## ESP32-C5 Devboard

We recommend using our custom hardware, which can be found in [this repo](https://codeberg.org/opentrafficmap/its-g5-receiver).
However, you can also use a ESP32-C5-WIFI6-KIT devboard.

If you are using our custom hardware, you can skip this section.

If you are using the devboard, make the following connections from the Ethernet module (left) to the ESP32 devboard (right):

```arduino
MOSI <-> GPIO7
MISO <-> GPIO2
SCLK <-> GPIO6
CS   <-> GPIO4
INT  <-> GPIO9
```

The ESP32-C5-WIFI6-KIT has a 100 nF capacitor on GPIO6, which will prevent the SPI communication from working.
This can be solved by de-soldering the zero-ohm resistor to the right of the 5V pin (named R39 in [the schematics](https://files.waveshare.com/wiki/ESP32-C5-WIFI6-KIT-NXRX/ESP32-C5-WIFI6-KIT-NXRX-Schematic.pdf)).
Alternatively, you can edit the sdkconfig to use a different pin.

Copy the correct sdkconfig file over the default one:

```sh
# For W5500
cp sdkconfig.proto-w5500 sdkconfig
# or alternatively, for ENC28J60:
cp sdkconfig.proto-enc28j60 sdkconfig
# or alternatively, for SPI EPPP link (with e.g. https://github.com/hn/esp32-spi-eppp-server as WiFi gateway):
cp sdkconfig.proto-spi-eppp sdkconfig
```

When switching between variants in a tree that has already been built, run `idf.py fullclean`
afterwards.  Which SPI Ethernet driver component is pulled in is decided when CMake configures the
project, so a stale build directory keeps the previous variant's driver and the build fails with
`fatal error: esp_eth_mac_w5500.h: No such file or directory`.  Deleting `dependencies.lock` is not
enough.

If you make any changes to the hardware/sdkconfig, make sure to set `HW_VARIANT` to `custom` in the sdkconfig,
otherwise you may receive incompatible OTA updates if you connect to the official MQTT server.

## Building

The `install.sh` script needs to be run once to install the ESP32 toolchain files:

```sh
esp-idf/install.sh
```

To enter the ESP-IDF environment, you need to source the `export.sh` script once per shell:

```sh
. esp-idf/export.sh
```

Afterwards, you can build the project:

```sh
idf.py build
```

To flash the firmware and monitor the output, you can use the following command:

```sh
idf.py -p /dev/ttyACM0 -b 921600 flash monitor -b 115200
```
