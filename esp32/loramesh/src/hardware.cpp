// hardware.cpp

// collects hardware-specific init code (other than UI) for all boards

#include "lib/tinySSBlib.h"
#include "hardware.h"

// ---------------------------------------------------------------------------
#if defined(TINYSSB_BOARD_HELTEC) || defined(TINYSSB_BOARD_HELTEC3)

#ifdef HAS_LORA

#ifdef USING_SX1262
  SX1262 radio = new Module(SS, DIO1, RST_LoRa, BUSY_LoRa);
#endif
#ifdef USING_SX1276
  SX1276 radio = new Module(SS, DI0, RST);
#endif
#ifdef USING_SX1278
  SX1278 radio = new Module(SS, DI0, RST);
#endif

#endif // HAS_LORA


void hw_init()
{
#ifdef HAS_LORA
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("RadioLib success!");
  } else {
    while (true) {
      Serial.print(F("RadioLib failed, code "));
      Serial.println(state);
      delay(2000);
    }
  }
#endif // HAS_LORA
}

#endif // HELTEC, HELTEC3

// ---------------------------------------------------------------------------
#ifdef TINYSSB_BOARD_T5GRAY

void hw_init()
{
}

#endif

// ---------------------------------------------------------------------------
#ifdef TINYSSB_BOARD_TBEAM

#include <axp20x.h>
#include <Wire.h>

AXP20X_Class axp;

#ifdef HAS_GPS
TinyGPSPlus gps;
HardwareSerial GPS(1);
#endif

#ifdef USE_RADIO_LIB
# ifdef USING_SX1262
    SX1262 radio_sx1262 = new Module(SS, DI0, RST); // , RADIO_BUSY_PIN);
# endif
# ifdef USING_SX1276
    SX1276 radio_sx1276 = new Module(SS, DI0, RST); // , RADIO_BUSY_PIN);
# endif

RadioChoice fused_radio;

#endif

void hw_init()
{
  Wire.begin(21, 22);

  SPI.end();
  SPI.begin(SCK, MISO, MOSI); // copied from T-Deck code
  
  if (axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
    Serial.println("AXP192 Begin FAIL");
  } else {
    // Serial.println("AXP192 Begin PASS");
#ifdef HAS_LORA
    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
#else
    axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF);
#endif
#ifdef HAS_GPS
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
#else
    axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);
#endif
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
#ifdef HAS_OLED
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON); // OLED
#else
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_OFF); // no OLED
#endif

#ifdef HAS_OLED
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high、
#endif
    
#ifdef HAS_GPS
    GPS.begin(9600, SERIAL_8N1, 34, 12);   //17-TX 18-RX
#endif
  }

  int state = radio_sx1262.begin();
  if (state == RADIOLIB_ERR_NONE) {
    fused_radio.is_62 = true;
    Serial.println("RadioLib sx1262 init success!");
  } else {
    Serial.printf("RadioLib sx1262 init did not work (code %d), trying sx1276\r\n", state);
    state = radio_sx1276.begin();
    if (state == RADIOLIB_ERR_NONE) {
      fused_radio.is_62 = false;
      Serial.println("RadioLib sx1276 init success!");
    } else while (true) {
      Serial.print(F("RadioLib for both sx1276 and sx1262 failed, code "));
      Serial.println(state);
      delay(2000);
    }
  }
  
}

#endif // _TBEAM

// ---------------------------------------------------------------------------
#ifdef TINYSSB_BOARD_TDECK

#ifdef USE_RADIO_LIB
# ifdef USING_SX1262
  SX1262 sx1262_radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN,
                                   RADIO_RST_PIN, RADIO_BUSY_PIN);
# endif
#endif

#if 0
TouchLib *touch = NULL;
uint8_t   touchAddress = GT911_SLAVE_ADDRESS2;

static void scanDevices(TwoWire *w)
{
    uint8_t err, addr;
    int nDevices = 0;
    // uint32_t start = 0;
    for (addr = 1; addr < 127; addr++) {
        // start = millis();
        w->beginTransmission(addr);
        delay(2);
        err = w->endTransmission();
        if (err == 0) {
            nDevices++;
            /*
            Serial.print("I2C device found at address 0x");
            if (addr < 16) {
                Serial.print("0");
            }
            Serial.print(addr, HEX);
            Serial.println(" !");
            */
            if (addr == GT911_SLAVE_ADDRESS2) {
                touchAddress = GT911_SLAVE_ADDRESS2;
                Serial.println("Touchpad - found GT911 Drv Slave address: 0x14");
            } else if (addr == GT911_SLAVE_ADDRESS1) {
                touchAddress = GT911_SLAVE_ADDRESS1;
                Serial.println("Touchpad - found GT911 Drv Slave address: 0x5D");
            }
        } else if (err == 4) {
          /*
            Serial.print("Unknow error at address 0x");
            if (addr < 16) {
                Serial.print("0");
            }
            Serial.println(addr, HEX);
          */
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
}
#endif


void hw_init()
{
  //! The board peripheral power control pin needs to
  //  be set to HIGH when using the peripheral
  pinMode(BOARD_POWERON, OUTPUT);
  digitalWrite(BOARD_POWERON, HIGH);

  //! Set CS on all SPI buses to high level during initialization
  pinMode(BOARD_SDCARD_CS, OUTPUT);
  pinMode(RADIO_CS_PIN, OUTPUT);
  pinMode(BOARD_TFT_CS, OUTPUT);

  digitalWrite(BOARD_SDCARD_CS, HIGH);
  digitalWrite(RADIO_CS_PIN, HIGH);
  digitalWrite(BOARD_TFT_CS, HIGH);

  pinMode(BOARD_SPI_MISO, INPUT_PULLUP);

  Wire.begin(BOARD_I2C_SDA, BOARD_I2C_SCL);
  SPI.end();
  SPI.begin(BOARD_SPI_SCK, BOARD_SPI_MISO, BOARD_SPI_MOSI); //SD
 
  pinMode(BOARD_BOOT_PIN, INPUT_PULLUP);
  pinMode(BOARD_TBOX_G02, INPUT_PULLUP);
  pinMode(BOARD_TBOX_G01, INPUT_PULLUP);
  pinMode(BOARD_TBOX_G04, INPUT_PULLUP);
  pinMode(BOARD_TBOX_G03, INPUT_PULLUP);

  //Wakeup touch chip
  pinMode(BOARD_TOUCH_INT, OUTPUT);
  digitalWrite(BOARD_TOUCH_INT, HIGH);

  // Set touch int input
  pinMode(BOARD_TOUCH_INT, INPUT); delay(20);

  // Two touch screens, the difference between them is the device address,
  // use ScanDevices to get the existing I2C address
  scanDevices(&Wire);
  touch = new TouchLib(Wire, BOARD_I2C_SDA, BOARD_I2C_SCL, touchAddress);
  touch->init();

  pinMode(BOARD_BL_PIN, OUTPUT);
}

#endif

// ---------------------------------------------------------------------------
#ifdef TINYSSB_BOARD_TWATCH

// TWATCH_S3

#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI();

#if TFT_BL !=  BOARD_BL_PIN
#  error "Not using the already configured T-Watch file, please remove <Arduino/libraries/TFT_eSPI> and replace with <lib/TFT_eSPI>, please do not click the upgrade library button when opening sketches in ArduinoIDE versions 2.0 and above, otherwise the original configuration file will be replaced !!!"
#  error "Not using the already configured T-Watch file, please remove <Arduino/libraries/TFT_eSPI> and replace with <lib/TFT_eSPI>, please do not click the upgrade library button when opening sketches in ArduinoIDE versions 2.0 and above, otherwise the original configuration file will be replaced !!!"
#endif

#include "XPowersLib.h"
XPowersAXP2101 power;

/*
TouchLib *touch = NULL;
uint8_t   touchAddress = GT911_SLAVE_ADDRESS2;
*/

SPIClass radioBus = SPIClass(HSPI);
SX1262 sx1262_radio = new Module(BOARD_RADIO_SS, BOARD_RADIO_DI01,
                                 BOARD_RADIO_RST, BOARD_RADIO_BUSY,
                                 radioBus);
// sensors (SensorLib)
#include <TouchDrvFT6X36.hpp>
#include <SensorBMA423.hpp>
#include <SensorPCF8563.hpp>
#include <SensorDRV2605.hpp>
TouchDrvFT6X36 sensor_touch;
SensorBMA423   sensor_accel;
SensorPCF8563  sensor_rtc;
SensorDRV2605  sensor_lspkr;

// ---------------------------------------------------------------------------

#define LEDC_BACKLIGHT_FREQ         1000
#define LEDC_BACKLIGHT_BIT_WIDTH    8

uint8_t brightness;

void setBrightness(uint8_t level)
{
    if (!level) {
        power.disableALDO2();
        tft.writecommand(0x10);  //display sleep
        // touch...setPowerMode(PMODE_MONITOR);
    }
    if (!brightness && level != 0) {
        power.enableALDO2();
        tft.writecommand(0x11);  //display wakeup
    }
    brightness = level;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
    ledcWrite(BOARD_TFT_BL, brightness);
#else
    ledcWrite(LEDC_BACKLIGHT_CHANNEL, brightness);
#endif
}

// ---------------------------------------------------------------------------

#ifndef GPSSerial
#define GPSSerial   Serial1
#endif

struct uBloxGnssModelInfo { // Structure to hold the module info (uses 341 bytes of RAM)
    char softVersion[30];
    char hardwareVersion[10];
    uint8_t extensionNo = 0;
    char extension[10][30];
};

static int getAck(uint8_t *buffer, uint16_t size, uint8_t requestedClass, uint8_t requestedID)
{
    uint16_t    ubxFrameCounter = 0;
    bool        ubxFrame = 0;
    uint32_t    startTime = millis();
    uint16_t    needRead;

    while (millis() - startTime < 800) {
        while (GPSSerial.available()) {
            int c = GPSSerial.read();
            switch (ubxFrameCounter) {
            case 0:
                if (c == 0xB5) {
                    ubxFrameCounter++;
                }
                break;
            case 1:
                if (c == 0x62) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 2:
                if (c == requestedClass) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 3:
                if (c == requestedID) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 4:
                needRead = c;
                ubxFrameCounter++;
                break;
            case 5:
                needRead |=  (c << 8);
                ubxFrameCounter++;
                break;
            case 6:
                if (needRead >= size) {
                    ubxFrameCounter = 0;
                    break;
                }
                if (GPSSerial.readBytes(buffer, needRead) != needRead) {
                    ubxFrameCounter = 0;
                } else {
                    return needRead;
                }
                break;

            default:
                break;
            }
        }
    }
    return 0;
}

static bool gpsProbe()
{
    uint8_t buffer[256];
    bool legacy_ubx_message = true;
    struct uBloxGnssModelInfo info ;
    //  Get UBlox GPS module version
    uint8_t cfg_get_hw[] =  {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};
    GPSSerial.write(cfg_get_hw, sizeof(cfg_get_hw));

    uint16_t len = getAck(buffer, 256, 0x0A, 0x04);
    if (len) {
        memset(&info, 0, sizeof(info));
        uint16_t position = 0;
        for (int i = 0; i < 30; i++) {
            info.softVersion[i] = buffer[position];
            position++;
        }

        for (int i = 0; i < 10; i++) {
            info.hardwareVersion[i] = buffer[position];
            position++;
        }

        while (len >= position + 30) {
            for (int i = 0; i < 30; i++) {
                info.extension[info.extensionNo][i] = buffer[position];
                position++;
            }
            info.extensionNo++;
            if (info.extensionNo > 9)
                break;
        }

        /*
        log_i("Module Info : ");
        log_i("Soft version: %s", info.softVersion);
        log_i("Hard version: %s", info.hardwareVersion);
        log_i("Extensions: %d", info.extensionNo);
        for (int i = 0; i < info.extensionNo; i++) {
            log_i("%s", info.extension[i]);
        }
        log_i("Model:%s", info.extension[2]);

        for (int i = 0; i < info.extensionNo; ++i) {
            if (!strncmp(info.extension[i], "OD=", 3)) {
                strcpy((char *)buffer, &(info.extension[i][3]));
                log_i("GPS Model: %s", (char *)buffer);
            }
        }
        */
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------

void hw_init()
{
  setCpuFrequencyMhz(160); // 240

  pinMode(BOARD_TOUCH_INT, INPUT);

  Wire.begin(BOARD_I2C_SDA, BOARD_I2C_SCL);
  Wire1.begin(BOARD_TOUCH_SDA, BOARD_TOUCH_SCL);
  
  power.init(Wire, BOARD_I2C_SDA, BOARD_I2C_SCL);

  power.setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_4V36);
  power.setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_900MA);
  power.setSysPowerDownVoltage(2600);

  power.setALDO1Voltage(3300); //! RTC VBAT , Don't change
  power.setALDO2Voltage(3300); //! TFT BACKLIGHT VDD , Don't change
  power.setALDO3Voltage(3300); //! Screen touch VDD , Don't change
  power.setALDO4Voltage(3300); //! Radio VDD , Don't change
  power.setBLDO2Voltage(3300); //! DRV2605
  power.setDC3Voltage(3300);   //! GPS Power
  power.enableDC3();

  //! No use
  power.disableDC2();
  // disableDC3();
  power.disableDC4();
  power.disableDC5();
  power.disableBLDO1();
  power.disableCPUSLDO();
  power.disableDLDO1();
  power.disableDLDO2();
  
  power.enableALDO1();  //! RTC VBAT
  power.enableALDO2();  //! TFT BACKLIGHT   VDD
  power.enableALDO3();  //! Screen touch VDD
  power.enableALDO4();  //! Radio VDD
  power.enableBLDO2();  //! drv2605 enable

  // Set the time of pressing the button to turn off
  power.setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
  // Set the button power-on press time
  power.setPowerKeyPressOnTime(XPOWERS_POWERON_1S);

  // It is necessary to disable the detection function of the TS pin
  // on the board without the battery temperature detection function,
  // otherwise it will cause abnormal charging
  power.disableTSPinMeasure();

  // Enable internal ADC detection
  power.enableBattDetection();
  power.enableVbusVoltageMeasure();
  power.enableBattVoltageMeasure();
  power.enableSystemVoltageMeasure();
  
  power.setChargingLedMode(XPOWERS_CHG_LED_OFF); //t-watch no chg led

  power.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);

  // Enable the required interrupt function
  power.enableIRQ(
    XPOWERS_AXP2101_BAT_INSERT_IRQ   | XPOWERS_AXP2101_BAT_REMOVE_IRQ   |   //BATTERY
    XPOWERS_AXP2101_VBUS_INSERT_IRQ  | XPOWERS_AXP2101_VBUS_REMOVE_IRQ  |   //VBUS
    XPOWERS_AXP2101_PKEY_SHORT_IRQ   | XPOWERS_AXP2101_PKEY_LONG_IRQ    |   //POWER KEY
    XPOWERS_AXP2101_BAT_CHG_DONE_IRQ | XPOWERS_AXP2101_BAT_CHG_START_IRQ    //CHARGE
  );

  // Clear all interrupt flags
  power.clearIrqStatus();

  // Set the precharge charging current
  power.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_50MA);
  // Set constant current charge current limit
  power.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_300MA);
  // Set stop charging termination current
  power.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);

  // Set charge cut-off voltage
  power.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V35);
  // Set RTC Battery voltage to 3.3V
  power.setButtonBatteryChargeVoltage(3300);
  power.enableButtonBatteryCharge();

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
  ledcAttach(BOARD_TFT_BL, LEDC_BACKLIGHT_FREQ, LEDC_BACKLIGHT_BIT_WIDTH);
#else
  ledcSetup(LEDC_BACKLIGHT_CHANNEL, LEDC_BACKLIGHT_FREQ, LEDC_BACKLIGHT_BIT_WIDTH);
  ledcAttachPin(BOARD_TFT_BL, LEDC_BACKLIGHT_CHANNEL);
#endif

    // ------------------------------------------------------------
    Serial.println("# FT6X36 init (touch sensor)");
    int res = sensor_touch.begin(Wire1, FT6X36_SLAVE_ADDRESS,
                                  BOARD_TOUCH_SDA, BOARD_TOUCH_SCL);
    if (!res)
        Serial.println("#   failed -- check your wiring");
    else {
        Serial.println("#   succeeded");
        sensor_touch.interruptTrigger(); //enable Interrupt
        // devices_probe |= WATCH_TOUCH_ONLINE;
    }
    // Wire.begin(BOARD_I2C_SDA, BOARD_I2C_SCL);
    // pinMode(BOARD_TOUCH_INT, INPUT); delay(20);
  
    // ------------------------------------------------------------
    Serial.println("# BMA423 init (acceleration)");
    res = sensor_accel.init(Wire);
    if (!res)
        Serial.println("#   failed -- check your wiring");
    else {
        Serial.println("#   succeeded");
        sensor_accel.setReampAxes(
                         SensorBMA423::REMAP_BOTTOM_LAYER_TOP_RIGHT_CORNER);
        sensor_accel.setStepCounterWatermark(1);
        // devices_probe |= WATCH_BMA_ONLINE;
    }

    // ------------------------------------------------------------
    Serial.println("# PCF8563 init (RTC)");
    res = sensor_rtc.init(Wire);
    if (!res)
        Serial.println("#   failed -- check your wiring");
    else {
        Serial.println("#   succeeded");
        sensor_rtc.disableCLK();   //Disable clock output ， Conserve Backup Battery Current Consumption
        sensor_rtc.hwClockRead();  //Synchronize RTC clock to system clock
        // devices_probe |= WATCH_RTC_ONLINE;
    }

    // ------------------------------------------------------------
    Serial.println("# DRV2605 init (loudspeaker)");
    res = sensor_lspkr.init(Wire);
    if (!res)
        Serial.println("#   failed -- check your wiring");
    else {
        Serial.println("#   succeeded");
        sensor_lspkr.selectLibrary(1);
        sensor_lspkr.setMode(DRV2605_MODE_INTTRIG);
        sensor_lspkr.useERM();
        sensor_lspkr.setWaveform(0, 15);  // play effect
        sensor_lspkr.setWaveform(1, 0);  // end waveform
        sensor_lspkr.run();
        // devices_probe |= WATCH_DRV_ONLINE;
    }

    // ------------------------------------------------------------
    Serial.println("# UBlox init (GPS)");
    GPSSerial.begin(38400, SERIAL_8N1, SHIELD_GPS_RX, SHIELD_GPS_TX);
    if (!gpsProbe()) {
        Serial.println("#   failed -- check your wiring");
        power.disableDC3();
    } else
        Serial.println("#   succeeded");

    tft.init();
    tft.setRotation(2);
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextFont(2);
    tft.drawString("< tinySSB >", 120, 120);
    setBrightness(50);
    delay(50);

    radioBus.begin(BOARD_RADIO_SCK, BOARD_RADIO_MISO, BOARD_RADIO_MOSI); 
}

#endif // TINYSSB_BOARD_TWATCH

// ---------------------------------------------------------------------------
#ifdef TINYSSB_BOARD_TWRIST

void hw_init()
{
}

#endif

// ---------------------------------------------------------------------------
#ifdef TINYSSB_BOARD_WLPAPER

#ifdef HAS_LORA
  SPIClass spi(SPI);
  SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE0);
# ifdef USING_SX1262
  SX1262 radio = new Module(RADIO_NSS, RADIO_DIO_1, RADIO_RESET, RADIO_BUSY, spi); // , spiSettings);
#endif

#endif // HAS_LORA

void hw_init()
{
  setCpuFrequencyMhz(240);

  /*
  // is this necessary, or helpful?
  pinMode(VEXT, INPUT); // OFF
  delay(50);
  pinMode(VEXT, OUTPUT); // ON
  digitalWrite(VEXT, LOW);
  delay(100);
  */

#ifdef HAS_LORA
  // begin(float freq = 434.0, float bw = 125.0, uint8_t sf = 9, uint8_t cr = 7, uint8_t syncWord = RADIOLIB_SX126X_SYNC_WORD_PRIVATE, int8_t power = 10, uint16_t preambleLength = 8, float tcxoVoltage = 1.6, bool useRegulatorLDO = false);
  spi.begin(LORA_CLK, LORA_MISO, LORA_MOSI, RADIO_NSS); //SCK, MISO, MOSI, SS
  int state = radio.begin(902.4, 125, 7);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("RadioLib success!");
  } else {
    while (true) {
      Serial.print(F("RadioLib failed, code "));
      Serial.println(state);
      delay(2000);
    }
  }
#endif // HAS_LORA
}

#endif // WIRELESS_PAPER

// ---------------------------------------------------------------------------

// eof
