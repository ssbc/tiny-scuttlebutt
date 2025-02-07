# Makefile

# ** usage:

# make BOARD=.. [SERIAL=/dev/tty..] {firmware} {flush|fullflush}

#    mandatory board, known values are: Heltec, Heltec3 (has more Flash),
#                      T5gray, TBeam, TDeck, TWatch, TWrist, WLpaper
#
#    optional serial device: we have some heuristics for setting
#                      it automatically, and loop through several
#                      devices in onw go (convenient for flashing
#                      multiple devices)


# ** examples:

# make BOARD=Heltec firmware     --> creates 'build/loramesh.ino.bin
# make BOARD=Heltec flash        --> flashes this binary into the app
#                                    partition, preserves the data

# make BOARD=Heltec fullflash    --> also rewrites the parition table
#                                    and erases all data content

# these two phases can be combined:

# make BOARD=Heltec firmware flash


# ** special flag for old TBeam:

#   make BOARD=TBeam FLAG=TBEAM_07 firmware flash


ifndef BOARD
  $(info ">> please specify a BOARD (Heltec, Heltec3, T5gray, TBeam, TDeck, TWatch, TWrist, WLpaper)")
  stop
endif

# ---------------------------------------------------------------------------
# project-specific flags and values

ESP32_CORE_VERSION=3.1.1  # our default platform
BAUD=921600
BOOTLOADER_OFFS=0x1000

ifeq ($(BOARD),Heltec)       # "v2"
  CHIP=esp32
  FQBN=esp32:esp32:heltec_wifi_lora_32_V2
  PARTITIONS_CSV=partitions-heltec.csv
  MAX_UPLOAD=1572864 # 1.5MB
  FLASH_SIZE=4MB
  FLAGS_BOARD=-DTINYSSB_BOARD_HELTEC

else ifeq ($(BOARD),Heltec3) # "v3"
  CHIP=esp32s3
  FQBN=esp32:esp32:heltec_wifi_lora_32_V3
  PARTITIONS_CSV=partitions-heltec.csv
  MAX_UPLOAD=1572864 # 1.5MB
  FLASH_SIZE=8MB     # !! more flash than Heltec2
  FLAGS_BOARD=-DTINYSSB_BOARD_HELTEC3

else ifeq ($(BOARD),T5gray)
  BAUD=460800
  CHIP=esp32
  FQBN=esp32:esp32:esp32
  PARTITIONS_CSV=partitions-t5gray.csv
  MAX_UPLOAD=1572864 # 1.5MB
  FLASH_SIZE=4MB
  FLAGS_BOARD=-DTINYSSB_BOARD_T5GRAY -DESP32 -DCORE_DEBUG_LEVEL=0 -DARDUINO_RUNNING_CORE=1 -DARDUINO_EVENT_RUNNING_CORE=1 -DARDUINO_USB_CDC_ON_BOOT=0

else ifeq ($(BOARD),TBeam)  # old sx1276 as well as newer sx1262 LoRa chip
  BAUD=460800
  CHIP=esp32
  FQBN=esp32:esp32:t-beam
  PARTITIONS_CSV=partitions-tbeam.csv
  MAX_UPLOAD=1572864 # 1.5MB
  FLASH_SIZE=4MB
  FLAGS_BOARD=-DTINYSSB_BOARD_TBEAM -DESP32 -DCORE_DEBUG_LEVEL=0 -DARDUINO=10607 -DARDUINO_TBEAM_USE_RADIO_SX1262 -DARDUINO_ARCH_ESP32 -DARDUINO_BOARD=TBEAM_USE_RADIO_SX1262 -DARDUINO_VARIANT=tbeam -DARDUINO_USB_CDC_ON_BOOT=0

else ifeq ($(BOARD),TDeck)
  CHIP=esp32s3
  FQBN=esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,CDCOnBoot=cdc
  PARTITIONS_CSV=partitions-tdeck.csv
  MAX_UPLOAD=2097152 # 2MB
  FLASH_SIZE=16MB
  FLAGS_BOARD=-DTINYSSB_BOARD_TDECK -DESP32 -DCORE_DEBUG_LEVEL=0 -DARDUINO_RUNNING_CORE=1 -DARDUINO_EVENT_RUNNING_CORE=1 -DBOARD_HAS_PSRAM -DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1 -DARDUINO_USB_MSC_ON_BOOT=0 -DARDUINO_USB_DFU_ON_BOOT=0

else ifeq ($(BOARD),TWatch)
  CHIP=esp32s3
  ESP32_CORE_VERSION=2.0.9
  BOOTLOADER_OFFS=0x0
  FQBN=esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,CDCOnBoot=cdc
  PARTITIONS_CSV=partitions-twatch.csv
  MAX_UPLOAD=2097152 # 2MB
  FLASH_SIZE=16MB
  FLAGS_BOARD=-DTINYSSB_BOARD_TWATCH -DUSING_TWATCH_S3=1

else ifeq ($(BOARD),TWrist)
  BAUD=460800
  CHIP=esp32
  FQBN=esp32:esp32:esp32
  PARTITIONS_CSV=partitions-twrist.csv
  MAX_UPLOAD=1572864 # 1.5MB
  FLASH_SIZE=4MB
  FLAGS_BOARD=-DTINYSSB_BOARD_TWRIST -DESP32 -DCORE_DEBUG_LEVEL=0 -DARDUINO_RUNNING_CORE=1 -DARDUINO_EVENT_RUNNING_CORE=1 -DARDUINO_USB_CDC_ON_BOOT=0

else ifeq ($(BOARD),WLpaper)
  CHIP=esp32s3
  FQBN=esp32:esp32:heltec_wireless_paper
  PARTITIONS_CSV=partitions-wlpaper.csv
  MAX_UPLOAD=1572864 # 1.5MB
  FLASH_SIZE=8MB
  FLAGS_BOARD=-DTINYSSB_BOARD_WLPAPER

else
  $(info ">> unknown board $(BOARD)")
  stop
endif

# ---------------------------------------------------------------------------
# automatically find the places where the arduino-cli executable etc sit

ARDUINO_DATA=$(shell arduino-cli config get directories |awk '/data/ {print $$2}';)
ARDUINO_ESP_DIR=$(ARDUINO_DATA)/packages/esp32
ARDUINO_ESP32_CORE=$(ARDUINO_ESP_DIR)/hardware/esp32/$(ESP32_CORE_VERSION)
# $(info ">> platform path is    $(ARDUINO_ESP32_CORE)")

ifeq ($(shell test -e $(ARDUINO_ESP32_CORE) && echo yes),yes)
    $(info ">> platform $(ESP32_CORE_VERSION) already installed")
else
    $(info ">> platform $(ESP32_CORE_VERSION) needs to be installed. Switching now ...")
    X=$(shell arduino-cli core install esp32:esp32@$(ESP32_CORE_VERSION);)
    $(info "$(X)")
endif

PLATFORM_TXT_FILE=$(shell find $(ARDUINO_ESP_DIR)/hardware/esp32 -name 'platform.txt' | head -1;)
# $(info PLATFORM_TXT_FILE  is   $(PLATFORM_TXT_FILE))

# ---------------------------------------------------------------------------

OS=$(shell uname;)
ifneq ($(OS), Darwin)
  ESPTOOL_EXT=.py
endif

ESPTOOL=$(shell find $(ARDUINO_ESP_DIR)/tools -name "esptool$(ESPTOOL_EXT)";)
BOOTAPP0PATH=$(shell find $(ARDUINO_ESP_DIR)/hardware -name "boot_app0.bin";)

MAIN=$(shell ls *.ino)

FLASH_BIN=firmware-$(BOARD).bin

# FLAGS_BOARD ... see above

FLAGS_PLATFORM=$(shell cat $(PLATFORM_TXT_FILE) | awk -F = '/^compiler.cpp.extra_flags/ {print $$2}';)

FLAGS_ALL=-DUTC_OFFSET=\"$(shell date +%z)\" -DUTC_COMPILE_TIME=\"$(shell date -u +'%b_%d_%Y_%H:%M:%S_UTC')\"

EXTRA_CPP_FLAGS=--build-property compiler.cpp.extra_flags="$(FLAGS_PLATFORM) $(FLAGS_ALL) $(FLAGS_BOARD) $(FLAG)"

# ---------------------------------------------------------------------------
# automatically find the serial ports for attached devices. Useful if
# multiple devices of the same BOARD type are to be flashed

ifndef SERIAL
  ifeq ($(BOARD),Heltec) # heltec LORA32 v2
    SERIAL = $(shell ls /dev/tty.usbserial-????)
  else ifeq ($(BOARD),Heltec3) # heltec LORA32 v3
    SERIAL = $(shell ls /dev/tty.usbserial-????)
  else ifeq ($(BOARD),T5gray)
    SERIAL = $(shell ls /dev/tty.usbserial-???????????)
  else ifeq ($(BOARD),TDeck)
    SERIAL = $(shell ls /dev/tty.usbmodem*)
  else ifeq ($(BOARD),TWatch)
    SERIAL = $(shell ls /dev/tty.usbmodem*)
  else ifeq ($(BOARD),TWrist)
    SERIAL = $(shell ls /dev/tty.usbserial-???????????)
  else ifeq ($(BOARD),WLpaper) # Heltec wireless paper 1.1
    SERIAL = $(shell ls /dev/tty.usbserial-????)
  else # T-Beam
    SERIAL= $(shell ls /dev/tty* | grep 'ACM\|USB\|wchusb')
  endif

  # ? kill any screen/miniterm program currently having the serial port open ?
  # $(foreach var,$(SERIAL), fuser -f $(var);) ...
endif

# ----------------------------------------------------------------------

all:
	$(info )
	$(info "usage:  make [BOARD=Heltec|Heltec3|T5gray|TBeam|TDeck|TWatch|TWrist|WLpaper] firmware|flash|fullflash")
	$(info )

monitor:
	$(info use CTRL-A CTRL-\ to quit the screen program)
	screen -h 1024 $(SERIAL) 115200

# ---------------------------------------------------------------------------



firmware:
	rm -f partitions.csv
	ln -s assets/$(PARTITIONS_CSV) partitions.csv
	mkdir -p build

	arduino-cli compile -v \
		--build-path ./build \
		--fqbn $(FQBN) \
		--build-property build.partitions=assets/$(PARTITIONS_CSV) \
		--build-property upload.maximum_size=$(MAX_UPLOAD) \
		--libraries ./lib/$(BOARD) \
		$(EXTRA_CPP_FLAGS) $(MAIN)


flash:
	$(foreach var,$(SERIAL), \
		$(ESPTOOL) \
		--chip $(CHIP) \
		--port $(var) --baud $(BAUD) \
		--before default_reset --after hard_reset \
		write_flash -z --flash_mode dio --flash_freq 80m \
		--flash_size $(FLASH_SIZE) \
		0x10000 ./build/$(MAIN).bin;)

$(FLASH_BIN): build/$(MAIN).bin
	$(ESPTOOL) \
		--chip $(CHIP) merge_bin -o ./build/$(FLASH_BIN) \
		--flash_mode dio --flash_freq 80m --flash_size $(FLASH_SIZE) \
		--fill-flash-size $(FLASH_SIZE) \
		$(BOOTLOADER_OFFS) ./build/$(MAIN).bootloader.bin \
		0x8000 ./build/$(MAIN).partitions.bin \
		0xe000 $(BOOTAPP0PATH) \
		0x10000 ./build/$(MAIN).bin

fullflash: $(FLASH_BIN)
	$(foreach var,$(SERIAL), \
		$(ESPTOOL) \
		--chip $(CHIP) \
		--port $(var) --baud $(BAUD) \
		--before default_reset --after hard_reset \
		write_flash -z --flash_mode dio --flash_freq 80m \
		--flash_size $(FLASH_SIZE) \
		0x0 ./build/$(FLASH_BIN);)

clean:
	rm -f *~
	rm -f partitions.csv
	rm -rf build/*

# eof
