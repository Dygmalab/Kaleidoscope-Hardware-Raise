#pragma once

#include <Arduino.h>

#define HARDWARE_IMPLEMENTATION Raise
#include "KeyboardioScanner.h"
#include "Kaleidoscope-HIDAdaptor-KeyboardioHID.h"

#define COLS 16
#define ROWS 5

#define CRGB(r,g,b) (cRGB){r, g, b}

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

class Raise {
 public:
  Raise(void);
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
  void showAnalogRGB(cRGB rgb);
  void enableHighPowerLeds(void);
  void enableScannerPower(void);
  void rebootBootloader();
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

  keydata_t leftHandState;
  keydata_t rightHandState;
  keydata_t previousLeftHandState;
  keydata_t previousRightHandState;

  typedef struct settings_t {
    uint8_t keyscan;
  } settings_t;

  static settings_t settings;

 private:
  static uint16_t settings_base_;
  static bool isLEDChanged;
  static KeyboardioScanner leftHand;
  static KeyboardioScanner rightHand;

  static keydata_t leftHandMask;
  static keydata_t rightHandMask;
};

// pullup resistor installed on the sides indicates ANSI
#define ANSI 1
#define ISO 0

#define FOCUS_HOOK_HARDWARE FOCUS_HOOK(Raise::focusHook,        \
                                           "hardware.keyscan\n" \
                                           "hardware.sled_ver\n" \
                                           "hardware.sled_current\n" \
                                           "hardware.joint\n" \
                                           "hardware.ansi_iso\n" \
                                           "hardware.side_ver")

#define SCANBIT(row,col) ((uint32_t)1 << ((row) * 8 + (7 - (col))))


// LEDS_PER_HAND defined in keyboardioscanner

#define LED_COUNT LEDS_LEFT + LEDS_RIGHT
#define LEDS_LEFT  LEDS_LEFT_KEYS  + LEDS_LEFT_UNDER
#define LEDS_RIGHT LEDS_RIGHT_KEYS + LEDS_RIGHT_UNDER

#define LEDS_LEFT_UNDER 31 // 31 includes 1 on LP
#define LEDS_RIGHT_UNDER 33 // 33 includes 1 on LP

#define LEDS_LEFT_KEYS 33 // 32 for ANSI, 33 is ISO
#define LEDS_RIGHT_KEYS 36


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

