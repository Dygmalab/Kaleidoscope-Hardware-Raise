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
  r0c0, r0c1, r0c2, r0c3, r0c4, r0c5, r0c6,         r0c9,  r0c10, r0c11, r0c12, r0c13, r0c14, r0c15, \
  r1c0, r1c1, r1c2, r1c3, r1c4, r1c5,               r1c9,  r1c10, r1c11, r1c12, r1c13, r1c14, r1c15, r1c8, \
  r2c0, r2c1, r2c2, r2c3, r2c4, r2c5,               r2c9,  r2c10, r2c11, r2c12, r2c13, r2c14, r2c15, \
  r3c0, r3c1, r3c2, r3c3, r3c4, r3c5,                      r3c10, r3c11, r3c12, r3c13, r3c14, r3c15, \
  r4c0, r4c1, r4c2, r4c3,             r4c4,         r4c11,               r4c12, r4c13, r4c14, r4c15, r4c8, \
                          r4c5, r4c6,                      r4c10, r4c9)                      \
  */
static constexpr uint8_t key_led_map[5][16] = {
  {5,  6,  7,  8,  9,  10, 11, XX,      XX,   11+LPH, 10+LPH, 9+LPH, 8+LPH, 7+LPH, 6+LPH, 5 +LPH}, //14
  {22, 23, 24, 25, 26, 27, XX, XX,      61+LPH, 27+LPH, 26+LPH, 25+LPH, 24+LPH, 23+LPH, 22+LPH, 60 +LPH}, //14
  {36, 37, 38, 39, 40, 41, XX, XX,      XX,   62+LPH, 41+LPH, 40+LPH, 39+LPH, 38+LPH, 37+LPH, 36 +LPH}, //13
  {44, 45, 46, 47, 48, 49, XX, XX,      XX, XX,   49+LPH, 48+LPH, 47+LPH, 46+LPH, 45+LPH, 44 +LPH}, //12
  {52, 53, 54, 55, 28, 29, 56, XX,      56+LPH, XX, XX, 55+LPH, 54+LPH, 53+LPH, 52+LPH, 63 +LPH}, //15  // 2 XX are for low profile - not sure what order
};

static constexpr uint8_t underglow_led_map[2][28] = {
    { XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX },
    { XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX, XX },
};

// rgb pwm pin definitions
int red[3] = { 3, 6, 10, };
int blu[3] = { 4, 8, 11, };
int grn[3] = { 5, 9, 12, };

void Raise::showAnalogRGB(cRGB rgb, int i)
{
    // invert as these are common anode, and make sure we reach 65535 to be able to turn fully off.
    analogWrite(red[i], ((256-pgm_read_byte(&gamma8[rgb.r])) << 8) -1 );
    analogWrite(grn[i], ((256-pgm_read_byte(&gamma8[rgb.g])) << 8) -1 );
    analogWrite(blu[i], ((256-pgm_read_byte(&gamma8[rgb.b])) << 8) -1 );
}

Raise::Raise(void) {

}

void Raise::enableScannerPower(void) {
    digitalWrite(0, HIGH);
    digitalWrite(1, HIGH);
}

// This lets the keyboard pull up to 1.6 amps from
// the host. That violates the USB spec. But it sure
// is pretty looking
void Raise::enableHighPowerLeds(void) {
  // PE6
  //    pinMode(7, OUTPUT);
  //    digitalWrite(7, LOW);
  //DDRE |= _BV(6);
  //PORTE &= ~_BV(6);

  // Set B4, the overcurrent check to an input with an internal pull-up
 // DDRB &= ~_BV(4);	// set bit, input
 // PORTB &= ~_BV(4);	// set bit, enable pull-up resistor



}

void Raise::setup(void) {
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  digitalWrite(0, LOW);
  digitalWrite(1, LOW);

  // arduino zero analogWrite(255) isn't fully on as its actually working with a 16bit counter and the mapping is a bit shift.
  // so change to 16 bit resolution to avoid the mapping and do the mapping ourselves in showAnalogRGB() to ensure LEDs can be
  // set fully off
  analogWriteResolution(16);
  for(int i = 0; i < 3; i ++)
      showAnalogRGB({0,0,0},i);

  delay(1000);
  enableScannerPower();

  // Consider not doing this until 30s after keyboard
  // boot up, to make it easier to rescue things
  // in case of power draw issues.
  enableHighPowerLeds();
  leftHandState.all = 0;
  rightHandState.all = 0;

  // initialise Wire of scanner - have to do this here to avoid problem with static object intialisation ordering
  twi_init();

  //TWBR = 72; // This is 100khz, 
  //TWBR = 12; // This is 400khz, which is the fastest we can drive the ATTiny
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

  for(int i = 0; i < 3; i ++)
      showAnalogRGB( {leftHand.ledData.leds[1].r, leftHand.ledData.leds[1].g, leftHand.ledData.leds[1].b} , i);

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

HARDWARE_IMPLEMENTATION KeyboardHardware;
