#pragma once

#include <Arduino.h>

#define HARDWARE_IMPLEMENTATION Raise
#include "KeyboardioScanner.h"
#include "Kaleidoscope-HIDAdaptor-KeyboardioHID.h"

#define COLS 16
#define ROWS 5

#define CRGB(r,g,b) (cRGB){r, g, b}

// adc pins for USB C CC pins
#define UFP_CC  A1
#define DFPL_CC A2
#define DFPR_CC A3

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

 private:
  static bool isLEDChanged;
  static KeyboardioScanner leftHand;
  static KeyboardioScanner rightHand;

  static keydata_t leftHandMask;
  static keydata_t rightHandMask;
};

#define FOCUS_HOOK_HARDWARE FOCUS_HOOK(Raise::focusHook,        \
                                           "hardware.keyscan\n" \
                                           "hardware.sled_ver\n" \
                                           "hardware.joint\n" \
                                           "hardware.cc\n" \
                                           "hardware.side_ver")

#define SCANBIT(row,col) ((uint32_t)1 << ((row) * 8 + (7 - (col))))


#ifndef DOXYGEN_SHOULD_SKIP_THIS


#define R0C0  SCANBIT(0, 0)
#define R0C1  SCANBIT(0, 1)
#define R0C2  SCANBIT(0, 2)
#define R0C3  SCANBIT(0, 3)
#define R0C4  SCANBIT(0, 4)
#define R0C5  SCANBIT(0, 5)
#define R0C6  SCANBIT(0, 6)
#define R0C7  SCANBIT(0, 7)
#define R1C0  SCANBIT(1, 0)
#define R1C1  SCANBIT(1, 1)
#define R1C2  SCANBIT(1, 2)
#define R1C3  SCANBIT(1, 3)
#define R1C4  SCANBIT(1, 4)
#define R1C5  SCANBIT(1, 5)
#define R1C6  SCANBIT(1, 6)
#define R1C7  SCANBIT(1, 7)
#define R2C0  SCANBIT(2, 0)
#define R2C1  SCANBIT(2, 1)
#define R2C2  SCANBIT(2, 2)
#define R2C3  SCANBIT(2, 3)
#define R2C4  SCANBIT(2, 4)
#define R2C5  SCANBIT(2, 5)
#define R2C6  SCANBIT(2, 6)
#define R2C7  SCANBIT(2, 7)
#define R3C0  SCANBIT(3, 0)
#define R3C1  SCANBIT(3, 1)
#define R3C2  SCANBIT(3, 2)
#define R3C3  SCANBIT(3, 3)
#define R3C4  SCANBIT(3, 4)
#define R3C5  SCANBIT(3, 5)
#define R3C6  SCANBIT(3, 6)
#define R3C7  SCANBIT(3, 7)

#define R0C8  SCANBIT(0, 0)
#define R0C9  SCANBIT(0, 1)
#define R0C10 SCANBIT(0, 2)
#define R0C11 SCANBIT(0, 3)
#define R0C12 SCANBIT(0, 4)
#define R0C13 SCANBIT(0, 5)
#define R0C14 SCANBIT(0, 6)
#define R0C15 SCANBIT(0, 7)
#define R1C8  SCANBIT(1, 0)
#define R1C9  SCANBIT(1, 1)
#define R1C10 SCANBIT(1, 2)
#define R1C11 SCANBIT(1, 3)
#define R1C12 SCANBIT(1, 4)
#define R1C13 SCANBIT(1, 5)
#define R1C14 SCANBIT(1, 6)
#define R1C15 SCANBIT(1, 7)
#define R2C8  SCANBIT(2, 0)
#define R2C9  SCANBIT(2, 1)
#define R2C10 SCANBIT(2, 2)
#define R2C11 SCANBIT(2, 3)
#define R2C12 SCANBIT(2, 4)
#define R2C13 SCANBIT(2, 5)
#define R2C14 SCANBIT(2, 6)
#define R2C15 SCANBIT(2, 7)
#define R3C8  SCANBIT(3, 0)
#define R3C9  SCANBIT(3, 1)
#define R3C10 SCANBIT(3, 2)
#define R3C11 SCANBIT(3, 3)
#define R3C12 SCANBIT(3, 4)
#define R3C13 SCANBIT(3, 5)
#define R3C14 SCANBIT(3, 6)
#define R3C15 SCANBIT(3, 7)


// LEDS_PER_HAND defined in keyboardioscanner
#define LED_COUNT LEDS_PER_HAND * 2 // total possible - not all connected


#define LED_PGDN 0
#define LED_PGUP 1
#define LED_BACKTICK 2
#define LED_PROG 3
#define LED_1 4
#define LED_Q 5
#define LED_A 6
#define LED_Z 7
#define LED_X 8
#define LED_S 9
#define LED_W 10
#define LED_2 11
#define LED_3 12
#define LED_E 13
#define LED_D 14
#define LED_C 15
#define LED_V 16
#define LED_F 17
#define LED_R 18
#define LED_4 19
#define LED_5 20
#define LED_T 21
#define LED_G 22
#define LED_B 23
#define LED_ESC 24
#define LED_TAB 25
#define LED_LED 26
#define LED_L_FN 27
#define LED_L_CTRL 28
#define LED_DEL 29
#define LED_CMD 30
#define LED_L_SHIFT 31
#define LED_R_SHIFT 32
#define LED_ALT 33
#define LED_SPACE 34
#define LED_CTRL 35
#define LED_R_FN 36
#define LED_ANY 37
#define LED_RETURN 38
#define LED_BUTTERFLY 39
#define LED_N 40
#define LED_H 41
#define LED_Y 42
#define LED_6 43
#define LED_7 44
#define LED_U 45
#define LED_J 46
#define LED_M 47
#define LED_COMMA 48
#define LED_K 49
#define LED_I 50
#define LED_8 51
#define LED_9 52
#define LED_O 53
#define LED_L 54
#define LED_PERIOD 55
#define LED_SLASH 56
#define LED_SEMICOLON 57
#define LED_P 58
#define LED_0 59
#define LED_NUM 60
#define LED_EQUALS 61
#define LED_APOSTROPHE 62
#define LED_MINUS 63

#endif /* DOXYGEN_SHOULD_SKIP_THIS */


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

#define KEYMAP_60(                                                                                     \
  r0c0, r0c1, r0c2, r0c3, r0c4, r0c5, r0c6,                r0c9,  r0c10, r0c11, r0c12, r0c13, r0c14, r0c15, \
  r1c0, r1c1, r1c2, r1c3, r1c4, r1c5,               r1c8,  r1c9,  r1c10, r1c11, r1c12, r1c13, r1c14, r1c15, \
  r2c0, r2c1, r2c2, r2c3, r2c4, r2c5,                      r2c9,  r2c10, r2c11, r2c12, r2c13, r2c14, r2c15, \
  r3c0, r3c1, r3c2, r3c3, r3c4, r3c5,                             r3c10, r3c11, r3c12, r3c13, r3c14, r3c15, \
  r4c0, r4c1, r4c2, r4c3, r4c4, r4c5,                             r4c10, r4c11, r4c12, r4c13, r4c14, r4c15, \
                          r4c6, r4c7,                             r4c8, r4c9)                      \
  {                                                                                                 \
    {r0c0, r0c1, r0c2, r0c3, r0c4, r0c5, r0c6, XXX , XXX,  r0c9, r0c10, r0c11, r0c12, r0c13, r0c14, r0c15}, \
    {r1c0, r1c1, r1c2, r1c3, r1c4, r1c5, XXX , XXX , r1c8, r1c9, r1c10, r1c11, r1c12, r1c13, r1c14, r1c15}, \
    {r2c0, r2c1, r2c2, r2c3, r2c4, r2c5, XXX , XXX , XXX,  r2c9, r2c10, r2c11, r2c12, r2c13, r2c14, r2c15}, \
    {r3c0, r3c1, r3c2, r3c3, r3c4, r3c5, XXX , XXX , XXX,  XXX,  r3c10, r3c11, r3c12, r3c13, r3c14, r3c15}, \
    {r4c0, r4c1, r4c2, r4c3, r4c4, XXX , r4c6, r4c7, r4c8, r4c9, r4c10, r4c11, r4c12, r4c13, r4c14, r4c15}, \
  }

