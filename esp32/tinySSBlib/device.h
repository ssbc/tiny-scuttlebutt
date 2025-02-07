// device.h

// tinySSB for ESP32, Lilygo T-Deck/T-Wrist version
// (c) 2023 <christian.tschudin@unibas.ch>

// ---------------------------------------------------------------------------
// hardware profile:

// #define HAS_BLE         // enable Bluetooth Low Energy
// #define HAS_BT          // the ESP32-3C has no Bluetooth
// #define HAS_LORA        // enable LoRa
// #define USE_RADIO_LIB

// #define HAS_UDP
// #define HAS_ ...

// ---------------------------------------------------------------------------

#ifdef TINYSSB_BOARD_HELTEC
// https://heltec.org/project/wifi-lora-32/
# define DEVICE_MAKE "Heltec LoRa32 v2"
# define HAS_BLE
# define HAS_LORA
# define HAS_OLED

# define USE_RADIO_LIB
# define USING_SX1276 // 868/915 MHz
# define radio sx1276_radio

//# define USING_SX1278 // 433 MHz

//# define USE_LORA_LIB

#endif // TINYSSB_BOARD_HELTEC

// ---------------------------------------------------------------------------

#ifdef TINYSSB_BOARD_HELTEC3
# define DEVICE_MAKE "Heltec LoRa32 v3"
# define HAS_BLE
# define HAS_LORA
# define HAS_OLED

# define USE_RADIO_LIB
# define USING_SX1262
# define radio sx1262_radio

//# define USE_LORA_LIB

#endif // TINYSSB_BOARD_HELTEC3

// ---------------------------------------------------------------------------

#ifdef TINYSSB_BOARD_T5GRAY
// https://www.lilygo.cc/products/t5-2-13inch-e-paper
# define DEVICE_MAKE "Lilygo T5-Grayscale"
# define HAS_BLE

#endif // TINYSSB_BOARD_T5GRAY

// ---------------------------------------------------------------------------

#ifdef TINYSSB_BOARD_TBEAM
// https://www.lilygo.cc/products/t-beam-v1-1-esp32-lora-module
# define DEVICE_MAKE "Lilygo T-Beam"
# define HAS_BLE
# define HAS_GPS
# define HAS_LORA
# define HAS_OLED

# define USE_RADIO_LIB

// try out both:
# include "RadioLib.h"
# define USING_SX1276 // 868/916 MHz
  extern SX1276 radio_sx1276;
# define USING_SX1262 // better Semtec chip, used in newer TBeams
  extern SX1262 radio_sx1262;

extern class RadioChoice {
public:
  int is_62;
#define _method0V(M)   { if (is_62) radio_sx1262.M(); else radio_sx1276.M(); }
#define _method0I(M)   { return is_62 ? radio_sx1262.M() : radio_sx1276.M(); }
#define _method1(M,P) { return is_62 ? radio_sx1262.M(P):radio_sx1276.M(P); }
  int setFrequency(float f)     _method1(setFrequency,f);
  int setBandwidth(int bw)      _method1(setBandwidth,bw);
  int setSpreadingFactor(int v) _method1(setSpreadingFactor,v);
  int setCodingRate(int v)      _method1(setCodingRate,v);
  int setSyncWord(int v)        _method1(setSyncWord,v);
  int setOutputPower(int v)     _method1(setOutputPower,v);
  int setCurrentLimit(int v)    _method1(setCurrentLimit,v);
  int setPreambleLength(int v)  _method1(setPreambleLength,v);
  int setCRC(int v)             _method1(setCRC,v);
  void setPacketReceivedAction(void (*v)(void)) _method1(setPacketReceivedAction,v);
  void standby()                _method0V(standby);
  int startReceive()            _method0I(startReceive);
  int getPacketLength()         _method0I(getPacketLength);
  int getRSSI()                 _method0I(getRSSI);
  int getSNR()                  _method0I(getSNR);
  int readData(unsigned char *buf, int len) {
    return is_62 ? radio_sx1262.readData(buf,len) : radio_sx1276.readData(buf,len);
  };
  int transmit(unsigned char *buf, int len, int a) {
    return is_62 ? radio_sx1262.transmit(buf,len,a) : radio_sx1276.transmit(buf,len,a);
  };
  int begin(void) {
    return is_62 ? radio_sx1262.begin() : radio_sx1276.begin();
  };
} fused_radio;

# define radio fused_radio

#endif // TINYSSB_BOARD_TBEAM

// ---------------------------------------------------------------------------

#ifdef TINYSSB_BOARD_TDECK
// https://www.lilygo.cc/products/t-deck

# define DEVICE_MAKE "Lilygo T-Deck"
# define HAS_BLE
# define HAS_LORA

# define USE_RADIO_LIB
# define USING_SX1262  // better Semtec chip, used in newer TBeams, TDeck
# define radio sx1262_radio

#endif // TINYSSB_BOARD_TDECK

// ---------------------------------------------------------------------------

#ifdef TINYSSB_BOARD_TWATCH
// https://www.lilygo.cc/products/t-watch-s3

#define USING_TWATCH_S3

# define DEVICE_MAKE "Lilygo T-Watch S3"
# define HAS_BLE
# define HAS_LORA

# define USE_RADIO_LIB
# define USING_SX1262
# define radio sx1262_radio

#endif // TINYSSB_BOARD_TWATCH

// ---------------------------------------------------------------------------

#ifdef TINYSSB_BOARD_TWRIST
// https://www.lilygo.cc/products/t-wrist-e-paper-1-54-inch-display
# define DEVICE_MAKE "Lilygo T-Wrist"
# define HAS_BLE

#endif // TINYSSB_BOARD_TWRIST

// ---------------------------------------------------------------------------

#ifdef TINYSSB_BOARD_WLPAPER
# define DEVICE_MAKE "Heltec Wireless Paper"
# define HAS_BLE
# define HAS_LORA
# define HAS_OLED

# define USE_RADIO_LIB
# define USING_SX1262
# define radio sx1262_radio

//# define USE_LORA_LIB

#endif // TINYSSB_BOARD_WLPAPER


// LoRa ----------------------------------------------------------------

#ifdef USE_RADIO_LIB
# include "RadioLib.h"

# ifdef USING_SX1262
   extern SX1262 sx1262_radio;
# endif
# ifdef USING_SX1276
  extern SX1276 sx1276_radio;
# endif

#endif

#ifdef USE_LORA_LIB
# include "LoRa.h"
#endif


// FS -----------------------------------------------------------------

// #include <SD.h>
// #include <LITTLEFS.h>
// #define MyFS LITTLEFS

#include <LittleFS.h>
#define MyFS LittleFS


// GPS ----------------------------------------------------------------

#ifdef HAS_GPS
# include <TinyGPS++.h>
  extern TinyGPSPlus gps;
#endif


// eof
