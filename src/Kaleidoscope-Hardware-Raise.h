#pragma once

#include <Arduino.h>

#define HARDWARE_IMPLEMENTATION kaleidoscope::hardware::dygma::Raise
#include "KeyboardioScanner.h"
#include "Kaleidoscope-HIDAdaptor-KeyboardioHID.h"

#include "FlashStorage.h"
#include "FlashAsEEPROM.h"

#define CRGB(r,g,b) (cRGB){r, g, b}

#include "kaleidoscope/Hardware.h"
#include "kaleidoscope/driver/Storage.h"

// adc pins for USB C CC pins - now unused
/*
#define UFP_CC  A1
#define DFPL_CC A2
#define DFPR_CC A3
*/

// led PWM pins pa9, pa8, pa15
#define PWM_R 3
#define PWM_G 5
#define PWM_B 4

// side power switch pa10
#define SIDE_POWER 1

namespace kaleidoscope {

namespace driver {
namespace storage {

class FlashAsEEPROMStorage {
 public:

  FlashAsEEPROMStorage() {}


  uint8_t read(int address) {
    return EEPROM.read(address);
  }
  void write(int address, uint8_t value) {
    return EEPROM.write(address, value);
  }
  void update(int address, uint8_t value) {
    return EEPROM.update(address, value);
  }
  uint16_t length() {
    return EEPROM.length();
  }

  template<typename T>
  static T& get(uint16_t address, T& t) {
    return EEPROM.get(address, t);
  }

  template<typename T>
  static T& put(uint16_t address, T& t) {
    EEPROM.put(address, t);
  }

  void commit() {
    EEPROM.commit();
  }
};

class DummyStorage {
 public:
  DummyStorage() {}

  template<typename T>
  static T& get(uint16_t offset, T& t) {
  }

  template<typename T>
  static const T& put(uint16_t offset, T& t) {
  }

  uint8_t read(int idx) {
  }

  void write(int idx, uint8_t val) {
  }

  void update(int idx, uint8_t val) {
  }

  uint16_t length() {
    return 0;
  }

  void commit() {}
};

}
}

namespace hardware {
namespace dygma {

KALEIDOSCOPE_HARDWARE_INVENTORY(dygma, Raise,
                                WITH_STORAGE(FlashAsEEPROMStorage));

#define XX 0xFF // off

class Raise: public kaleidoscope::Hardware<Raise> {
  friend class kaleidoscope::Hardware<Raise>;
 public:
  Raise(void);

  static constexpr byte matrix_rows = 5;
  static constexpr byte matrix_columns = 16;
  static constexpr uint8_t left_keys        = 33; // 32 for ANSI
  static constexpr uint8_t right_keys       = 36;
  static constexpr uint8_t left_underglow   = 30;
  static constexpr uint8_t right_underglow  = 32;
  static constexpr uint8_t huble_leds       = 1;
  static constexpr uint8_t led_count = left_keys + right_keys + left_underglow + right_underglow + huble_leds; // 131 + 1 for the huble



  void syncLeds(void);
  void setCrgbAt(byte row, byte col, cRGB color);
  void setCrgbAt(uint8_t i, cRGB crgb);
  cRGB getCrgbAt(uint8_t i);
  uint8_t getLedIndex(byte row, byte col);
  static bool focusHook(const char *command);

  void scanMatrix(void);
  void readMatrix(void);
  void actOnMatrixScan(void);
  void setup();
  void updateHubleLED();
  void enableHighPowerLeds(void);
  void enableSidePower(void);
  void disableSidePower(void);
  void rebootBootloader();
  /* focus calls */
  uint8_t getSidePower();
  uint8_t leftCRCErrors();
  uint8_t rightCRCErrors();
  uint8_t leftVersion();
  uint8_t rightVersion();
  uint8_t leftSLEDVersion();
  uint8_t rightSLEDVersion();
  uint8_t rightSLEDCurrent();
  uint8_t leftSLEDCurrent();
  void setSLEDCurrent(uint8_t);
  void setSidePower(uint8_t);
  uint8_t readANSI_ISO();
  uint8_t rightKeyscanInterval();
  uint8_t leftKeyscanInterval();
  void setKeyscanInterval(uint8_t);
  uint16_t readJoint();

  void attachToHost();
  void detachFromHost();

  boolean ledPowerFault(void);

  /* Key masking
   * -----------
   *
   * There are situations when one wants to ignore key events for a while, and
   * mask them out. These functions help do that. In isolation, they do nothing,
   * plugins and the core firmware is expected to make use of these.
   *
   * See `handleKeyswitchEvent` in the Kaleidoscope sources for a use-case.
   */
  void maskKey(byte row, byte col);
  void unMaskKey(byte row, byte col);
  bool isKeyMasked(byte row, byte col);
  void maskHeldKeys(void);

  /** Key switch states
   *
   * These methods offer a way to peek at the key switch states, for those cases
   * where we need to deal with the state closest to the hardware. Some methods
   * offer a way to check if a key is pressed, others return the number of
   * pressed keys.
   */
  /**
   * Check if a key is pressed at a given position.
   *
   * @param row is the row the key is located at in the matrix.
   * @param col is the column the key is located at in the matrix.
   *
   * @returns true if the key is pressed, false otherwise.
   */
  bool isKeyswitchPressed(byte row, byte col);
  /**
   * Check if a key is pressed at a given position.
   *
   * @param keyIndex is the key index, as calculated by `keyIndex`.
   *
   * @note Key indexes start at 1, not 0!
   *
   * @returns true if the key is pressed, false otherwise.
   */
  bool isKeyswitchPressed(uint8_t keyIndex);
  /**
   * Check the number of key switches currently pressed.
   *
   * @returns the number of keys pressed.
   */
  uint8_t pressedKeyswitchCount();

  keydata_t leftHandState;
  keydata_t rightHandState;
  keydata_t previousLeftHandState;
  keydata_t previousRightHandState;

  typedef struct settings_t {
    uint16_t keyscan;
  } settings_t;

  static settings_t settings;
  

 protected:
  kaleidoscope::driver::storage::FlashAsEEPROMStorage storage_;

 private:
  static uint16_t settings_base_;
  static bool isLEDChangedHuble;
  static uint8_t isLEDChangedLeft[LED_BANKS];
  static uint8_t isLEDChangedRight[LED_BANKS];
  static uint8_t ansi_iso;
  static KeyboardioScanner leftHand;
  static KeyboardioScanner rightHand;
  static bool lastLeftOnline;
  static bool lastRightOnline;
  static bool sidePower;

  static cRGB hubleLED;

  static keydata_t leftHandMask;
  static keydata_t rightHandMask;
  void initialiseSides();
};
}
}
}

// pullup resistor installed on the sides indicates ANSI
#define ANSI 1
#define ISO 0





#define KEYMAP_STACKED(                                                 \
               r0c0, r0c1, r0c2, r0c3, r0c4, r0c5, r0c6,                \
               r1c0, r1c1, r1c2, r1c3, r1c4, r1c5, r1c6,                \
               r2c0, r2c1, r2c2, r2c3, r2c4, r2c5,                      \
               r3c0, r3c1, r3c2, r3c3, r3c4, r3c5, r2c6,                \
               r0c7, r1c7, r2c7, r3c7,                                  \
               r3c6,                                                    \
                                                                        \
               r0c9,  r0c10, r0c11, r0c12, r0c13, r0c14, r0c15,         \
               r1c9,  r1c10, r1c11, r1c12, r1c13, r1c14, r1c15,         \
                      r2c10, r2c11, r2c12, r2c13, r2c14, r2c15,         \
               r2c9,  r3c10, r3c11, r3c12, r3c13, r3c14, r3c15,         \
               r3c8,  r2c8,  r1c8, r0c8,                                \
               r3c9)                                                    \
  {                                                                     \
    {r0c0, r0c1, r0c2, r0c3, r0c4, r0c5, r0c6, r0c7, r0c8, r0c9, r0c10, r0c11, r0c12, r0c13, r0c14, r0c15}, \
    {r1c0, r1c1, r1c2, r1c3, r1c4, r1c5, r1c6, r1c7, r1c8, r1c9, r1c10, r1c11, r1c12, r1c13, r1c14, r1c15}, \
    {r2c0, r2c1, r2c2, r2c3, r2c4, r2c5, r2c6, r2c7, r2c8, r2c9, r2c10, r2c11, r2c12, r2c13, r2c14, r2c15}, \
    {r3c0, r3c1, r3c2, r3c3, r3c4, r3c5, r3c6, r3c7, r3c8, r3c9, r3c10, r3c11, r3c12, r3c13, r3c14, r3c15}, \
  }

// r3c0 (key 20) will not be placed for ANSI.
#define KEYMAP_60(                                                                                     \
  r0c0, r0c1, r0c2, r0c3, r0c4, r0c5, r0c6,                r0c9,  r0c10, r0c11, r0c12, r0c13, r0c14, r0c15, \
  r1c0, r1c1, r1c2, r1c3, r1c4, r1c5,               r1c8,  r1c9,  r1c10, r1c11, r1c12, r1c13, r1c14, r1c15, \
  r2c0, r2c1, r2c2, r2c3, r2c4, r2c5,                      r2c9,  r2c10, r2c11, r2c12, r2c13, r2c14, r2c15, \
  r3c0, r3c1, r3c2, r3c3, r3c4, r3c5, r3c6,                       r3c10, r3c11, r3c12, r3c13, r3c14, r3c15, \
  r4c0, r4c1, r4c2, r4c3, r4c4,                                   r4c10, r4c11, r4c12, r4c13, r4c14, r4c15, \
                          r4c6, r4c7,                             r4c8, r4c9)                      \
  {                                                                                                 \
    {r0c0, r0c1, r0c2, r0c3, r0c4, r0c5, r0c6, XXX , XXX,  r0c9, r0c10, r0c11, r0c12, r0c13, r0c14, r0c15}, \
    {r1c0, r1c1, r1c2, r1c3, r1c4, r1c5, XXX , XXX , r1c8, r1c9, r1c10, r1c11, r1c12, r1c13, r1c14, r1c15}, \
    {r2c0, r2c1, r2c2, r2c3, r2c4, r2c5, XXX , XXX , XXX,  r2c9, r2c10, r2c11, r2c12, r2c13, r2c14, r2c15}, \
    {r3c0, r3c1, r3c2, r3c3, r3c4, r3c5, r3c6, XXX , XXX,  XXX,  r3c10, r3c11, r3c12, r3c13, r3c14, r3c15}, \
    {r4c0, r4c1, r4c2, r4c3, r4c4, XXX , r4c6, r4c7, r4c8, r4c9, r4c10, r4c11, r4c12, r4c13, r4c14, r4c15}, \
  }

#include "kaleidoscope/hardware/key_indexes.h"
