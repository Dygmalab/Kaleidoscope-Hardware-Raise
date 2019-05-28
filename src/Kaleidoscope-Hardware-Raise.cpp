#include <Kaleidoscope.h>
#include <KeyboardioHID.h>
#include <Kaleidoscope-EEPROM-Settings.h>
#define RAISE_WATCHDOG
#ifdef RAISE_WATCHDOG
#include <Adafruit_SleepyDog.h>
#endif

namespace kaleidoscope {
namespace hardware {
namespace dygma {

KeyboardioScanner Raise::leftHand(0);
KeyboardioScanner Raise::rightHand(1);
bool Raise::lastLeftOnline = false;
bool Raise::lastRightOnline = false;

bool Raise::isLEDChanged = true;
uint8_t Raise::ansi_iso = ANSI;
keydata_t Raise::leftHandMask;
keydata_t Raise::rightHandMask;
cRGB Raise::hubleLED = {0,0,0};
uint16_t Raise::settings_base_;

Raise::settings_t Raise::settings = {
  .keyscan = 50
};

#define LKEY Raise::left_keys
// these are zero indexed led numbers from sled matrix
// maps rows and columns to the keyboard led number (0->LED_COUNT-1)
// 19 is missing for ANSI
static constexpr uint8_t key_led_map[2][5][16] = {
  {
    // ISO
    {0,  1,  2,  3,  4,  5,  6,  XX,      XX,   6+LKEY, 5+LKEY, 4+LKEY, 3+LKEY, 2+LKEY, 1+LKEY, 0+LKEY},
    {7,  8,  9,  10, 11, 12, XX, XX,      14+LKEY, 13+LKEY, 12+LKEY, 11+LKEY, 10+LKEY, 9+LKEY, 8+LKEY, 7 +LKEY},
    {13, 14, 15, 16, 17, 18, XX, XX,      XX,   21+LKEY, 20+LKEY, 19+LKEY, 18+LKEY, 17+LKEY, 16+LKEY, 15 +LKEY},
    {19, 20, 21, 22, 23, 24, 25, XX,      XX, XX,   27+LKEY, 26+LKEY, 25+LKEY, 24+LKEY, 23+LKEY, 22 +LKEY}, // ISO
    {26, 27, 28, 29, 30, XX, 31, 32,      35+LKEY, 34+LKEY, 33+LKEY, 32+LKEY, 31+LKEY, 30+LKEY, 29+LKEY, 28+LKEY}, 
  },
  {
    // ANSI
    {0,  1,  2,  3,  4,  5,  6,  XX,      XX,   6+LKEY, 5+LKEY, 4+LKEY, 3+LKEY, 2+LKEY, 1+LKEY, 0+LKEY},
    {7,  8,  9,  10, 11, 12, XX, XX,      14+LKEY, 13+LKEY, 12+LKEY, 11+LKEY, 10+LKEY, 9+LKEY, 8+LKEY, 7 +LKEY},
    {13, 14, 15, 16, 17, 18, XX, XX,      XX,   21+LKEY, 20+LKEY, 19+LKEY, 18+LKEY, 17+LKEY, 16+LKEY, 15 +LKEY},
    {19, XX, 21, 22, 23, 24, 25, XX,      XX, XX,   27+LKEY, 26+LKEY, 25+LKEY, 24+LKEY, 23+LKEY, 22 +LKEY}, // ANSI
    {26, 27, 28, 29, 30, XX, 31, 32,      35+LKEY, 34+LKEY, 33+LKEY, 32+LKEY, 31+LKEY, 30+LKEY, 29+LKEY, 28+LKEY}, 
    }
};

// maps keyboard led number (0->LED_COUNT-1) to the SLED led number (0-LPH-1 on left side  and LPH to LPH*2-1 on the right side)
// LPH comes from keyboardioscanner.h - leds per hand = 72 defined by the size of the buffer used to transfer data to sides
static constexpr uint8_t led_map[2][Raise::led_count] = {
  // ISO
  {
    // left side - 33 keys includes LP
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 68, 69,

    // right side - 36 keys includes LP
    0+LPH, 1+LPH, 2+LPH, 3+LPH, 4+LPH, 5+LPH, 6+LPH, 7+LPH, 8+LPH, 9+LPH, 10+LPH, 11+LPH, 12+LPH, 13+LPH, 14+LPH, 15+LPH, 16+LPH, 17+LPH, 18+LPH, 19+LPH,
    20+LPH, 21+LPH, 22+LPH, 23+LPH, 24+LPH, 25+LPH, 26+LPH, 27+LPH, 28+LPH, 29+LPH, 30+LPH, 31+LPH, 32+LPH, 33+LPH, 68+LPH, 69+LPH,

    // left under glow - 30
    34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,

    // right underglow - 32
    34+LPH, 35+LPH, 36+LPH, 37+LPH, 38+LPH, 39+LPH, 40+LPH, 41+LPH, 42+LPH, 43+LPH, 44+LPH, 45+LPH, 46+LPH, 47+LPH, 48+LPH, 49+LPH, 50+LPH, 51+LPH,
    52+LPH, 53+LPH, 54+LPH, 55+LPH, 56+LPH, 57+LPH, 58+LPH, 59+LPH, 60+LPH, 61+LPH, 62+LPH, 63+LPH, 64+LPH, 65+LPH,
  },
  // ANSI
  {
    // left side - 32 keys includes LP: key 19 is missing for ANSI layout
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 20, XX, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 68, 69,

    // right side - 36 keys includes LP
    0+LPH, 1+LPH, 2+LPH, 3+LPH, 4+LPH, 5+LPH, 6+LPH, 15+LPH, 8+LPH, 9+LPH, 10+LPH, 11+LPH, 12+LPH, 13+LPH, 14+LPH, 7+LPH, 16+LPH, 17+LPH, 18+LPH, 19+LPH,
    20+LPH, 21+LPH, 22+LPH, 23+LPH, 24+LPH, 25+LPH, 26+LPH, 27+LPH, 28+LPH, 29+LPH, 30+LPH, 31+LPH, 32+LPH, 33+LPH, 68+LPH, 69+LPH,

    // left under glow - 30
    34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,

    // right underglow - 32
    34+LPH, 35+LPH, 36+LPH, 37+LPH, 38+LPH, 39+LPH, 40+LPH, 41+LPH, 42+LPH, 43+LPH, 44+LPH, 45+LPH, 46+LPH, 47+LPH, 48+LPH, 49+LPH, 50+LPH, 51+LPH,
    52+LPH, 53+LPH, 54+LPH, 55+LPH, 56+LPH, 57+LPH, 58+LPH, 59+LPH, 60+LPH, 61+LPH, 62+LPH, 63+LPH, 64+LPH, 65+LPH,
  }
};


void Raise::updateHubleLED()
{
    // invert as these are common anode, and make sure we reach 65535 to be able to turn fully off.
    analogWrite(PWM_R, ((256-pgm_read_byte(&gamma8[hubleLED.r])) << 8) -1 );
    analogWrite(PWM_G, ((256-pgm_read_byte(&gamma8[hubleLED.g])) << 8) -1 );
    analogWrite(PWM_B, ((256-pgm_read_byte(&gamma8[hubleLED.b])) << 8) -1 );
}

Raise::Raise(void) {

}

void Raise::enableScannerPower(void) {
    pinMode(SIDE_POWER, OUTPUT);
    digitalWrite(SIDE_POWER, HIGH);
}


void Raise::setup(void) {
  // first set all pins to outputs and low - EMC considerations
  // not serial pins
  for (int pin = 2; pin < 27; pin ++)
  {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  // not USB pins
  for (int pin = 30; pin < 43; pin ++)
  {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  pinMode(SIDE_POWER, OUTPUT);
  digitalWrite(SIDE_POWER, LOW);

  // arduino zero analogWrite(255) isn't fully on as its actually working with a 16bit counter and the mapping is a bit shift.
  // so change to 16 bit resolution to avoid the mapping and do the mapping ourselves in updateHubleLED() to ensure LEDs can be
  // set fully off
  analogWriteResolution(16);
  updateHubleLED();

  //while(analogRead(UFP_CC) < 100) // should be about 150. If it's 0 then we are powered through one of the side ports
  //    updateHubleLED({100,0,0});

  delay(10);
  enableScannerPower();
  delay(500); // wait for sides to power up and finish bootloader

  // Consider not doing this until 30s after keyboard
  // boot up, to make it easier to rescue things
  // in case of power draw issues.
  leftHandState.all = 0;
  rightHandState.all = 0;

  // initialise Wire of scanner - have to do this here to avoid problem with static object intialisation ordering
  twi_init();
  
  settings_base_ = ::EEPROMSettings.requestSlice(sizeof(settings));

  // If keyscan is max, assume that EEPROM is uninitialized, and store the
  // defaults.
  uint16_t keyscan;
  KeyboardHardware.storage().get(settings_base_, keyscan);
  if (keyscan == 0xffff) {
    KeyboardHardware.storage().put(settings_base_, settings);
    KeyboardHardware.storage().commit();
  }

  KeyboardHardware.storage().get(settings_base_, settings);
  initialiseSides();
 
  // for now get ANSI/ISO once at boot - will require sides to be plugged in at power
  uint8_t l_ansi_iso = leftHand.readANSI_ISO();
  uint8_t r_ansi_iso = rightHand.readANSI_ISO();

  // setup ansi_iso variable, this will affect led mapping - defaults to ISO if nothing reported
  if(l_ansi_iso == ANSI || r_ansi_iso == ANSI)
    ansi_iso = ANSI;
  else 
    ansi_iso = ISO;

  #ifdef RAISE_WATCHDOG
  int countdownMS = Watchdog.enable(500); // milliseconds. 100 stops serial print from working...
  #endif

}

void Raise::initialiseSides()
{
  leftHand.setKeyscanInterval(settings.keyscan);
  rightHand.setKeyscanInterval(settings.keyscan);
  // force resync of LEDs
  isLEDChanged = true;
}

// i is number from 0 -> LED_COUNT - 1
void Raise::setCrgbAt(uint8_t i, cRGB crgb) {

  // prevent reading off the end of the led_map array
  if(i >= led_count)
    return;

  // huble LED
  if(i == led_count - 1)
  {
    isLEDChanged |= !(hubleLED.r == crgb.r && hubleLED.g == crgb.g && hubleLED.b == crgb.b);
    hubleLED = crgb;
    return;
  }

  // get the SLED index
  uint8_t sled_num = led_map[ansi_iso][i];
  if( sled_num < LPH) {
    cRGB oldColor = getCrgbAt(sled_num);
    leftHand.ledData.leds[ sled_num ] = crgb;
    isLEDChanged |= !(oldColor.r == crgb.r && oldColor.g == crgb.g && oldColor.b == crgb.b);
  }
  else if( sled_num < 2 * LPH) {
    cRGB oldColor = getCrgbAt(sled_num);
    rightHand.ledData.leds[ sled_num - LPH ] = crgb;
    isLEDChanged |= !(oldColor.r == crgb.r && oldColor.g == crgb.g && oldColor.b == crgb.b);
  } else {
    // TODO(anyone):
    // how do we want to handle debugging assertions about crazy user
    // code that would overwrite other memory?
  }
}

// sets LED given row and col
void Raise::setCrgbAt(byte row, byte col, cRGB color) {
  setCrgbAt(key_led_map[ansi_iso][row][col], color);
}

// returns keyboard LED index
uint8_t Raise::getLedIndex(byte row, byte col) {
  return key_led_map[ansi_iso][row][col];
}

// i is number from 0 -> LED_COUNT - 1
// returns LED colour given the keyboard LED index
cRGB Raise::getCrgbAt(uint8_t i) {
  // prevent reading off the end of the led_map array
  if(i >= led_count)
    return {0, 0, 0};

  uint8_t sled_num = led_map[ansi_iso][i];
  if (sled_num < LPH) {
    return leftHand.ledData.leds[ sled_num ];
  } else if (sled_num < 2 * LPH) {
    return rightHand.ledData.leds[ sled_num - LPH ];
  } else {
    return {0, 0, 0};
  }
}

void Raise::syncLeds() {
  if (!isLEDChanged)
    return;

  // left and right sides
  for(int i = 0; i < LED_BANKS; i ++)
  {
      leftHand.sendLEDData();
      rightHand.sendLEDData();
  }

  // huble
  updateHubleLED();

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
    // if ANSI, then swap r3c0 and r3c1 to match the PCB
    if(ansi_iso == ANSI)
        if((leftHandState.rows[3] & (1 << 0)) ^ leftHandState.rows[3] & (1 << 1)) // only swap if bits are different
        {
            leftHandState.rows[3] ^= (1 << 0); // flip the bit
            leftHandState.rows[3] ^= (1 << 1); // flip the bit
        }
  }

  if (rightHand.readKeys()) {
    rightHandState = rightHand.getKeyData();
    // if ANSI, then swap r1c0 and r2c0 to match the PCB
    if(ansi_iso == ANSI)
        if((rightHandState.rows[1] & (1 << 0)) ^ rightHandState.rows[2] & (1 << 0))
        {
            rightHandState.rows[1] ^= (1 << 0);
            rightHandState.rows[2] ^= (1 << 0);
        }
  }

  // if a side has just been replugged, initialse it
  if(leftHand.online && !lastLeftOnline || rightHand.online && !lastRightOnline)
    initialiseSides();

  // store previous state of whether the sides are plugged in
  lastLeftOnline = leftHand.online;
  lastRightOnline = rightHand.online;

}



void Raise::actOnMatrixScan() {
  for (byte row = 0; row < 5; row++) {
    for (byte col = 0; col < 8; col++) {

      uint8_t keynum = (row * 8) + (col);

      uint8_t keyState;
      
      // left
      keyState = (bitRead(previousLeftHandState.all, keynum) << 0) |
                 (bitRead(leftHandState.all, keynum) << 1);
      handleKeyswitchEvent(Key_NoKey, row,  col, keyState);

      // right
      keyState = (bitRead(previousRightHandState.all, keynum) << 0) |
                 (bitRead(rightHandState.all, keynum) << 1);
      handleKeyswitchEvent(Key_NoKey, row, (15 - col), keyState);
    }
  }
}


void Raise::scanMatrix() {
  readMatrix();
  actOnMatrixScan();
  #ifdef RAISE_WATCHDOG
  Watchdog.reset();
  #endif
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

// these need checking
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

// this works at least for the left side
bool Raise::isKeyswitchPressed(byte row, byte col) {
  if (col >= 8) {
    return (bitRead(rightHandState.rows[row], 15 - col ) != 0);
  } else {
    return (bitRead(leftHandState.rows[row], col) != 0);
  }
}

bool Raise::isKeyswitchPressed(uint8_t keyIndex) {
  keyIndex--;
  return isKeyswitchPressed(keyIndex / COLS, keyIndex % COLS);
}

uint8_t Raise::pressedKeyswitchCount() {
  uint8_t count = 0;

  for (uint8_t i = 0; i < ROWS * COLS; i++) {
    count += bitRead(leftHandState.all, i) + bitRead(rightHandState.all, i);
  }

  return count;
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

// all of this because the Hardware implementation can't also be of type Plugin - so no Focus hooks are available.
uint16_t Raise::readJoint(){
  return rightHand.readJoint();
}

uint8_t Raise::leftVersion() {
    return leftHand.readVersion();
}

uint8_t Raise::rightVersion() {
    return rightHand.readVersion();
}

uint8_t Raise::rightSLEDVersion() {
    return rightHand.readSLEDVersion();
}

uint8_t Raise::leftSLEDVersion() {
    return leftHand.readSLEDVersion();
}

uint8_t Raise::readLeftSLEDCurrent() {
    return leftHand.readSLEDCurrent();
}

uint8_t Raise::readRightSLEDCurrent() {
    return rightHand.readSLEDCurrent();
}

uint8_t Raise::readANSI_ISO() {
    return ansi_iso;
}

void Raise::setSLEDCurrent(uint8_t current) {
    rightHand.setSLEDCurrent(current);
    leftHand.setSLEDCurrent(current);
}

uint8_t Raise::readRightKeyscanInterval() {
  return rightHand.readKeyscanInterval();
}

uint8_t Raise::readLeftKeyscanInterval() {
  return leftHand.readKeyscanInterval();
}

void Raise::setKeyscanInterval(uint8_t interval) {
  settings.keyscan = interval;
  KeyboardHardware.storage().put(settings_base_, settings);
  KeyboardHardware.storage().commit();
  rightHand.setKeyscanInterval(interval);
  leftHand.setKeyscanInterval(interval);
}

}
}
}

HARDWARE_IMPLEMENTATION KeyboardHardware;
