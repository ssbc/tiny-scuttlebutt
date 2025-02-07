# How to flash a tinySSB firmware to your ESP32 device

## Unzip the collection of firmware binaries

```
% unzip tinySSB-esp32-firmware.zip
```

## Make sure you know your device

| Name    | Description                                 | Chip     | Size  |
| ------- | ------------------------------------------- | -------- | ----- |
| Heltec  | Heltec "WiFi LoRa 32(V2)"                   | esp32    | 4MB   |
| Heltec3 | Heltec "WiFi LoRa 32(V3)"                   | esp32s3  | 8MB   |
| TBeam   | Lilygo "T-Beam ESP32 LoRa Wireless Module"  | esp32    | 4MB   |
| TWatch  | Lilygo "T-Watch S3"                         | esp32s3  | 16MB  |
| WLpaper | Heltec "Wireless Paper"                     | esp32s3  | 8MB   |


## Use the "Name" to locate the firmwares for your device

E.f. for the "T-Watch S3", look for either of

```
tinySSB-esp32-firmware/firmware-TWatch-full.bin
tinySSB-esp32-firmware/firmware-TWatch-update.bin
```

## Pick the right firmware

The ```full``` version is necessary if you bring tinySSB to your
device for the first time. For this case a new partition table is
included in the firmware and will be written to your device, along
with a bootloader and the executable tinySSB code. Moreover, all data
on the device is erased, which can take several minutes.

The ```update``` version is useful if you have already installed
tinySSB (once or multiple times). In this case only the executable
code needs to be flashed, which does not take that much time.


## Locate the COM (serial USB) port

Connect your device to the USB port (make sure you have a "USB data
cable", not just a "USB charger cable"). On your computer, find the
name of the serial port to which your device is connected.


## Note the Chip

In the table above, your specific chip name needs to be picked.


## Launch the software to flash your device

The canonical tool to flash your device is the ```esptool```.

If you are **updating**, the following example applies:

```
% esptool --chip $(CHIP) --port $(PORT) --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size $(FLASH_SIZE) 0x10000 $(FLASH_BIN_UPDATE)
```

where the placeholders in uppercase have to be replaced with your
specific information.

If you are initializing your device for a **full** flashing, the
following example applies:

```
% esptool --chip $(CHIP) --port $(PORT) --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size $(FLASH_SIZE) 0x0 $(FLASH_BIN_FULL)
```

where the placeholders in uppercase have to be replaced with your
specific information. Note that the address of the second last
parameter is 0, in this "full" case.


## Problem - flashing a "busy device"

Flashing a connected device when it is operating normally is usually
easy.

If the code is buggy and you device boots and crashes in repeated
manner, then this becomes more challenging. Search the web for tips
how to put your device into a special "flash mode". On the T-Watch S3,
for example, a tiny switch (under the battery) has to be pressed
before powering on the device.

----
