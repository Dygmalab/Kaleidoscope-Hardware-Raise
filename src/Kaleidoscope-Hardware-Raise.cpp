#include <Kaleidoscope.h>
#include <KeyboardioHID.h>
//#include <avr/wdt.h>

KeyboardioScanner Raise::leftHand(0);
KeyboardioScanner Raise::rightHand(3);
bool Raise::isLEDChanged = true;
keydata_t Raise::leftHandMask;
keydata_t Raise::rightHandMask;

#define XX 0xFF // off

/*
#define KEYMAP_60(                                                                                     \
  r0c0, r0c1, r0c2, r0c3, r0c4, r0c5, r0c6,                r0c9,  r0c10, r0c11, r0c12, r0c13, r0c14, r0c15, \
  r1c0, r1c1, r1c2, r1c3, r1c4, r1c5,               r1c9,  r1c10, r1c11, r1c12, r1c13, r1c14, r1c15, r1c8, \
  r2c0, r2c1, r2c2, r2c3, r2c4, r2c5,                      r2c9,  r2c10, r2c11, r2c12, r2c13, r2c14, r2c15, \
  r3c0, r3c1, r3c2, r3c3, r3c4, r3c5,                             r3c10, r3c11, r3c12, r3c13, r3c14, r3c15, \
  r4c0, r4c1, r4c2, r4c3,             r4c4,                r4c11,        r4c12, r4c13, r4c14, r4c15, r4c8, \
                          r4c5, r4c6,                      r4c10, r4c9)                      \
#define KEYMAP_60(                                                                                     \
  r0c0, r0c1, r0c2, r0c3, r0c4, r0c5, r0c6,                r0c9,  r0c10, r0c11, r0c12, r0c13, r0c14, r0c15, \
  r1c0, r1c1, r1c2, r1c3, r1c4, r1c5,               r1c8,  r1c9,  r1c10, r1c11, r1c12, r1c13, r1c14, r1c15, \
  r2c0, r2c1, r2c2, r2c3, r2c4, r2c5,                      r2c9,  r2c10, r2c11, r2c12, r2c13, r2c14, r2c15, \
  r3c0, r3c1, r3c2, r3c3, r3c4, r3c5,                             r3c10, r3c11, r3c12, r3c13, r3c14, r3c15, \
  r4c0, r4c1, r4c2, r4c3,             r4c4,                r4c10,        r4c11, r4c12, r4c13, r4c14, r4c15, \
                          r4c6, r4c7,                      r4c8, r4c9)                      \
  */

// LPH comes from keyboardioscanner.h - leds per hand = 72 defined by the size of the buffer used to transfer data to sides
// these are zero indexed
static constexpr uint8_t key_led_map[5][16] = {
  {0,  1,  2,  3,  4,  5,  6,  XX,      XX,   6+LPH, 5+LPH, 4+LPH, 3+LPH, 2+LPH, 1+LPH, 0+LPH},
  {7,  8,  9,  10, 11, 12, XX, XX,      14+LPH, 13+LPH, 12+LPH, 11+LPH, 10+LPH, 9+LPH, 8+LPH, 7 +LPH},
  {13, 14, 15, 16, 17, 18, XX, XX,      XX,   21+LPH, 20+LPH, 19, 18+LPH, 17+LPH, 16+LPH, 15 +LPH},
  {19, 20, 21, 22, 23, 24, 25, XX,      XX, XX,   27+LPH, 26+LPH, 25+LPH, 24+LPH, 23+LPH, 22 +LPH},
  {26, 27, 28, 29, 30, 68, 69, XX,      69+LPH, 68+LPH, 33+LPH, 32+LPH, 31+LPH, 30+LPH, 29+LPH, 28+LPH}, 
};

static constexpr uint8_t underglow_led_map[2][28] = {
    { XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX },
    { XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX },
};

void Raise::showAnalogRGB(cRGB rgb)
{
    // invert as these are common anode, and make sure we reach 65535 to be able to turn fully off.
    analogWrite(PWM_R, ((256-pgm_read_byte(&gamma8[rgb.r])) << 8) -1 );
    analogWrite(PWM_G, ((256-pgm_read_byte(&gamma8[rgb.g])) << 8) -1 );
    analogWrite(PWM_B, ((256-pgm_read_byte(&gamma8[rgb.b])) << 8) -1 );
}

Raise::Raise(void) {

}

void Raise::enableScannerPower(void) {
    pinMode(SIDE_POWER, OUTPUT);
    digitalWrite(SIDE_POWER, HIGH);
}


void Raise::setup(void) {
  pinMode(SIDE_POWER, OUTPUT);
  digitalWrite(SIDE_POWER, LOW);


  // arduino zero analogWrite(255) isn't fully on as its actually working with a 16bit counter and the mapping is a bit shift.
  // so change to 16 bit resolution to avoid the mapping and do the mapping ourselves in showAnalogRGB() to ensure LEDs can be
  // set fully off
  analogWriteResolution(16);
  showAnalogRGB({0,0,0});

  while(analogRead(UFP_CC) < 100) // should be about 150. If it's 0 then we are powered through one of the side ports
      showAnalogRGB({100,0,0});

  delay(1000);
  enableScannerPower();

  // Consider not doing this until 30s after keyboard
  // boot up, to make it easier to rescue things
  // in case of power draw issues.
  leftHandState.all = 0;
  rightHandState.all = 0;

  // initialise Wire of scanner - have to do this here to avoid problem with static object intialisation ordering
  twi_init();
}

void Raise::setCrgbAt(uint8_t i, cRGB crgb) {
  if( i < LEDS_PER_HAND) {
    cRGB oldColor = getCrgbAt(i);
    leftHand.ledData.leds[i] = crgb;
    isLEDChanged |= !(oldColor.r == crgb.r && oldColor.g == crgb.g && oldColor.b == crgb.b);
  }
  else if( i < 2 * LEDS_PER_HAND) {
    cRGB oldColor = getCrgbAt(i);
    rightHand.ledData.leds[i-LEDS_PER_HAND] = crgb;
    isLEDChanged |= !(oldColor.r == crgb.r && oldColor.g == crgb.g && oldColor.b == crgb.b);
  }
  else if(i == XX ) {
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
  if (i < LEDS_PER_HAND) {
    return leftHand.ledData.leds[i];
  } else if (i < 2*LEDS_PER_HAND) {
    return rightHand.ledData.leds[i - LEDS_PER_HAND];
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

  // show on the huble LED the same colour as the firsts led on left side
  showAnalogRGB( {leftHand.ledData.leds[0].r, leftHand.ledData.leds[0].g, leftHand.ledData.leds[0].b});

  isLEDChanged = false;
}

boolean Raise::ledPowerFault() {
    return false;
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
  //wdt_enable(WDTO_120MS);

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

void Raise::detachFromHost() {
  //https://github.com/arduino/ArduinoCore-samd/blob/master/cores/arduino/USB/SAMD21_USBDevice.h#L60-L61
  USBDevice.detach();
}

void Raise::attachToHost() {
  USBDevice.attach();
}

HARDWARE_IMPLEMENTATION KeyboardHardware;
