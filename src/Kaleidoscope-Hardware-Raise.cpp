#include <Kaleidoscope.h>
#include <KeyboardioHID.h>
#include <avr/wdt.h>

KeyboardioScanner Raise::leftHand(0);
KeyboardioScanner Raise::rightHand(3);
bool Raise::isLEDChanged = true;
keydata_t Raise::leftHandMask;
keydata_t Raise::rightHandMask;

#define XX 0xFF // off

/*
#define KEYMAP_60(                                                                                     \
  r0c0, r0c1, r0c2, r0c3, r0c4, r0c5, r0c6,         r0c9,  r0c10, r0c11, r0c12, r0c13, r0c14, r0c15, \
  r1c0, r1c1, r1c2, r1c3, r1c4, r1c5,               r1c9,  r1c10, r1c11, r1c12, r1c13, r1c14, r1c15, r1c8, \
  r2c0, r2c1, r2c2, r2c3, r2c4, r2c5,               r2c9,  r2c10, r2c11, r2c12, r2c13, r2c14, r2c15, \
  r3c0, r3c1, r3c2, r3c3, r3c4, r3c5,                      r3c10, r3c11, r3c12, r3c13, r3c14, r3c15, \
  r4c0, r4c1, r4c2, r4c3,             r4c4,         r4c11,               r4c12, r4c13, r4c14, r4c15, r4c8, \
                          r4c5, r4c6,                      r4c10, r4c9)                      \
  */
static constexpr uint8_t key_led_map[5][16] = {
  {0,  1,  2,  3,  4,  5,  6,  XX,   XX, 32, 33, 34, 35, 36, 37, 38 }, //14
  {7,  8,  9,  10, 11, 12, XX, XX,   46, 39, 40, 41, 42, 43, 44, 45 }, //14
  {13, 14, 15, 16, 17, 18, XX, XX,   XX, 47, 48, 49, 50, 51, 52, 53 }, //13
  {19, 20, 21, 22, 23, 24, XX, XX,   XX, XX, 54, 55, 56, 57, 58, 59 }, //12
  {25, 26, 27, 28, 29, 30, 31, XX,    65, 66, 67, 60, 61, 62, 63, 64 }, //15
  //{25, 26, 27, 28, 29, 30, 31, XX,   60, 61, 62, 63, 64, 65, 66, 67 }, //15
};

static constexpr uint8_t underglow_led_map[2][28] = {
    { 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59 },
    { 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98 },
};

Raise::Raise(void) {

}

void Raise::enableScannerPower(void) {
  // PC7
  //pinMode(13, OUTPUT);
  //digitalWrite(13, HIGH);
  // Turn on power to the LED net
  DDRC |= _BV(7);
  PORTC |= _BV(7);

}

// This lets the keyboard pull up to 1.6 amps from
// the host. That violates the USB spec. But it sure
// is pretty looking
void Raise::enableHighPowerLeds(void) {
  // PE6
  //    pinMode(7, OUTPUT);
  //    digitalWrite(7, LOW);
  DDRE |= _BV(6);
  PORTE &= ~_BV(6);

  // Set B4, the overcurrent check to an input with an internal pull-up
  DDRB &= ~_BV(4);	// set bit, input
  PORTB &= ~_BV(4);	// set bit, enable pull-up resistor



}

void Raise::setup(void) {
  wdt_disable();
  delay(100);
  enableScannerPower();

  // Consider not doing this until 30s after keyboard
  // boot up, to make it easier to rescue things
  // in case of power draw issues.
  enableHighPowerLeds();
  leftHandState.all = 0;
  rightHandState.all = 0;

  TWBR = 72; // This is 100khz, which is the fastest we can drive the ATTiny
  //TWBR = 12; // This is 400khz, which is the fastest we can drive the ATTiny
}


/*

#define LEFT_KEYS 32
#define LEFT_UNDERGLOW_LEDS 14 + 16 + 2

#define RIGHT_KEYS 36
#define RIGHT_UNDERGLOW_LEDS 16 + 16 + 2
*/
void Raise::setCrgbAt(uint8_t i, cRGB crgb) {
  if (i < LEFT_KEYS) {  // left keys
    cRGB oldColor = getCrgbAt(i);
    isLEDChanged |= !(oldColor.r == crgb.r && oldColor.g == crgb.g && oldColor.b == crgb.b);

    leftHand.ledData.leds[i] = crgb;
  } else if (i < LEFT_KEYS + RIGHT_KEYS) { // right keys
    cRGB oldColor = getCrgbAt(i);
    isLEDChanged |= !(oldColor.r == crgb.r && oldColor.g == crgb.g && oldColor.b == crgb.b);

    rightHand.ledData.leds[i - LEFT_KEYS] = crgb;

  } else if (i < LEFT_KEYS + RIGHT_KEYS + LEFT_UNDERGLOW_LEDS) { // left under
    cRGB oldColor = getCrgbAt(i);
    isLEDChanged |= !(oldColor.r == crgb.r && oldColor.g == crgb.g && oldColor.b == crgb.b);

    leftHand.ledData.leds[i - RIGHT_KEYS] = crgb;
     
  } else if (i < LEFT_KEYS + RIGHT_KEYS + LEFT_UNDERGLOW_LEDS + RIGHT_UNDERGLOW_LEDS) { // right under
    cRGB oldColor = getCrgbAt(i);
    isLEDChanged |= !(oldColor.r == crgb.r && oldColor.g == crgb.g && oldColor.b == crgb.b);
    rightHand.ledData.leds[i - (LEFT_KEYS + LEFT_UNDERGLOW_LEDS)] = crgb;
  } else if (i == XX ) {
    // do nothing with missing leds
  } else {
    // TODO(anyone):
    // how do we want to handle debugging assertions about crazy user
    // code that would overwrite other memory?
  }
}

void Raise::setCrgbAt(byte row, byte col, cRGB color) {
  setCrgbAt(key_led_map[row][col], color);
}

uint8_t Raise::getLedIndex(byte row, byte col) {
  return key_led_map[row][col];
}

cRGB Raise::getCrgbAt(uint8_t i) {
  if (i < LEFT_KEYS) {
    return leftHand.ledData.leds[i];
  } else if (i < LEFT_KEYS + RIGHT_KEYS) {
    return rightHand.ledData.leds[i - LEFT_KEYS] ;
  } else if (i < LEFT_KEYS + RIGHT_KEYS + LEFT_UNDERGLOW_LEDS) {
    return leftHand.ledData.leds[i - RIGHT_KEYS] ;
  } else if (i < LEFT_KEYS + RIGHT_KEYS + LEFT_UNDERGLOW_LEDS + RIGHT_UNDERGLOW_LEDS ) {
    return rightHand.ledData.leds[i - (LEFT_KEYS + LEFT_UNDERGLOW_LEDS)] ;
  } else {
    return {0, 0, 0};
  }
}

void Raise::syncLeds() {
  if (!isLEDChanged)
    return;

  for(int i = 0; i < LED_BANKS; i ++)
  {
      leftHand.sendLEDData();
      rightHand.sendLEDData();
  }


  isLEDChanged = false;
}

boolean Raise::ledPowerFault() {
  if (PINB & _BV(4)) {
    return true;
  } else {
    return false;
  }
}

void debugKeyswitchEvent(keydata_t state, keydata_t previousState, uint8_t keynum, uint8_t row, uint8_t col) {
/*
  if (bitRead(state.all, keynum) != bitRead(previousState.all, keynum)) {
    Serial.print("Looking at row ");
    Serial.print(row);
    Serial.print(", col ");
    Serial.print(col);
    Serial.print(" key # ");
    Serial.print(keynum);
    Serial.print(" ");
    Serial.print(bitRead(previousState.all, keynum));
    Serial.print(" -> ");
    Serial.print(bitRead(state.all, keynum));
    Serial.println();
  }
  */
}


void Raise::readMatrix() {
  //scan the Keyboard matrix looking for connections
  previousLeftHandState = leftHandState;
  previousRightHandState = rightHandState;

  if (leftHand.readKeys()) {
    leftHandState = leftHand.getKeyData();
  }

  if (rightHand.readKeys()) {
    rightHandState = rightHand.getKeyData();
  }
}



void Raise::actOnMatrixScan() {
  for (byte row = 0; row < 5; row++) {
    for (byte col = 0; col < 8; col++) {

      uint8_t keynum = (row * 8) + (col);

      uint8_t keyState = (bitRead(previousLeftHandState.all, keynum) << 0) |
                         (bitRead(leftHandState.all, keynum) << 1);
      handleKeyswitchEvent(Key_NoKey, row,  col, keyState);

      keyState = (bitRead(previousRightHandState.all, keynum) << 0) |
                 (bitRead(rightHandState.all, keynum) << 1);

      handleKeyswitchEvent(Key_NoKey, row, (15 - col), keyState);
    }
  }
}


void Raise::scanMatrix() {
  readMatrix();
  actOnMatrixScan();
}

void Raise::rebootBootloader() {
  // Set the magic bits to get a Caterina-based device
  // to reboot into the bootloader and stay there, rather
  // than run move onward
  //
  // These values are the same as those defined in
  // Caterina.c

  uint16_t bootKey = 0x7777;
  uint16_t *const bootKeyPtr = reinterpret_cast<uint16_t *>(0x0800);

  // Stash the magic key
  *bootKeyPtr = bootKey;

  // Set a watchdog timer
  wdt_enable(WDTO_120MS);

  while (1) {} // This infinite loop ensures nothing else
  // happens before the watchdog reboots us
}

void Raise::maskKey(byte row, byte col) {
  if (row >= ROWS || col >= COLS)
    return;

  if (col >= 8) {
    rightHandMask.rows[row] |= 1 << (7 - (col - 8));
  } else {
    leftHandMask.rows[row] |= 1 << (7 - col);
  }
}

void Raise::unMaskKey(byte row, byte col) {
  if (row >= ROWS || col >= COLS)
    return;

  if (col >= 8) {
    rightHandMask.rows[row] &= ~(1 << (7 - (col - 8)));
  } else {
    leftHandMask.rows[row] &= ~(1 << (7 - col));
  }
}

bool Raise::isKeyMasked(byte row, byte col) {
  if (row >= ROWS || col >= COLS)
    return false;

  if (col >= 8) {
    return rightHandMask.rows[row] & (1 << (7 - (col - 8)));
  } else {
    return leftHandMask.rows[row] & (1 << (7 - col));
  }
}

void Raise::maskHeldKeys(void) {
  memcpy(leftHandMask.rows, leftHandState.rows, sizeof(leftHandMask));
  memcpy(rightHandMask.rows, rightHandState.rows, sizeof(rightHandMask));
}

HARDWARE_IMPLEMENTATION KeyboardHardware;
