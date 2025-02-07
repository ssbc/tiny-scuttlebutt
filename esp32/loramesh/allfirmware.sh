#!/bin/bash

# allfirmware.sh

DIR_NAME=tinySSB-esp32-firmware

mkdir -p ${DIR_NAME}
echo "firmware for tinySSB, compiled <`date -u`>" >${DIR_NAME}/README.md
cp assets/HOWTO_FLASH.md ${DIR_NAME}

# work in batches for the same ESP32 platform (2.0.9 and 3.1.1):
PLATFORM209_LIST="TWatch"
PLATFORM311_LIST="Heltec Heltec3 TBeam WLpaper"

for i in ${PLATFORM311_LIST}; do
    make BOARD=$i firmware
    cp build/firmware-* ${DIR_NAME}
done

for i in ${PLATFORM209_LIST}; do
    make BOARD=$i firmware
    cp build/firmware-* ${DIR_NAME}
done

zip -r ${DIR_NAME}.zip ${DIR_NAME}

# eof
