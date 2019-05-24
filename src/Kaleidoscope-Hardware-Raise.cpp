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
bool Raise::isLEDChanged = true;
uint8_t Raise::ansi_iso = ANSI;
keydata_t Raise::leftHandMask;
keydata_t Raise::rightHandMask;
cRGB Raise::hubleLED = {0,0,0};
uint16_t Raise::settings_base_;

Raise::settings_t Raise::settings = {
  .keyscan = 50
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
  leftHand.setKeyscanInterval(settings.keyscan);
  rightHand.setKeyscanInterval(settings.keyscan);
 
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
