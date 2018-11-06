#include <Kaleidoscope.h>
#include <KeyboardioHID.h>
#include <Kaleidoscope-EEPROM-Settings.h>
//#include <avr/wdt.h>

KeyboardioScanner Raise::leftHand(0);
KeyboardioScanner Raise::rightHand(1);
bool Raise::isLEDChanged = true;
keydata_t Raise::leftHandMask;
keydata_t Raise::rightHandMask;

Raise::settings_t Raise::settings = {
  .keyscan = 40
};

uint16_t Raise::settings_base_;

#define XX 0xFF // off


// these are zero indexed led numbers from sled matrix
// maps rows and columns to the keyboard led number (0->LED_COUNT-1)
// 19 is missing for ANSI
static constexpr uint8_t key_led_map[5][16] = {
  {0,  1,  2,  3,  4,  5,  6,  XX,      XX,   6+LEDS_LEFT_KEYS, 5+LEDS_LEFT_KEYS, 4+LEDS_LEFT_KEYS, 3+LEDS_LEFT_KEYS, 2+LEDS_LEFT_KEYS, 1+LEDS_LEFT_KEYS, 0+LEDS_LEFT_KEYS},
  {7,  8,  9,  10, 11, 12, XX, XX,      14+LEDS_LEFT_KEYS, 13+LEDS_LEFT_KEYS, 12+LEDS_LEFT_KEYS, 11+LEDS_LEFT_KEYS, 10+LEDS_LEFT_KEYS, 9+LEDS_LEFT_KEYS, 8+LEDS_LEFT_KEYS, 7 +LEDS_LEFT_KEYS},
  {13, 14, 15, 16, 17, 18, XX, XX,      XX,   21+LEDS_LEFT_KEYS, 20+LEDS_LEFT_KEYS, 19+LEDS_LEFT_KEYS, 18+LEDS_LEFT_KEYS, 17+LEDS_LEFT_KEYS, 16+LEDS_LEFT_KEYS, 15 +LEDS_LEFT_KEYS},
  {XX, 20, 21, 22, 23, 24, 25, XX,      XX, XX,   27+LEDS_LEFT_KEYS, 26+LEDS_LEFT_KEYS, 25+LEDS_LEFT_KEYS, 24+LEDS_LEFT_KEYS, 23+LEDS_LEFT_KEYS, 22 +LEDS_LEFT_KEYS},
  {26, 27, 28, 29, 30, XX, 31, 32,      35+LEDS_LEFT_KEYS, 34+LEDS_LEFT_KEYS, 33+LEDS_LEFT_KEYS, 32+LEDS_LEFT_KEYS, 31+LEDS_LEFT_KEYS, 30+LEDS_LEFT_KEYS, 29+LEDS_LEFT_KEYS, 28+LEDS_LEFT_KEYS}, 
};


// maps keyboard led number (0->LED_COUNT-1) to the SLED led number (0-LPH-1 on left side  and LPH to LPH*2-1 on the right side)
// LPH comes from keyboardioscanner.h - leds per hand = 72 defined by the size of the buffer used to transfer data to sides
static constexpr uint8_t led_map[LED_COUNT] = {
    // left side - 32/33 keys (ANSI/ISO) includes LP
    // 19 is missing for ANSI layout
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 68, 69,

    // right side - 36 keys includes LP
    0+LPH, 1+LPH, 2+LPH, 3+LPH, 4+LPH, 5+LPH, 6+LPH, 7+LPH, 8+LPH, 9+LPH, 10+LPH, 11+LPH, 12+LPH, 13+LPH, 14+LPH, 15+LPH, 16+LPH, 17+LPH, 18+LPH, 19+LPH, 20+LPH, 21+LPH, 22+LPH, 23+LPH, 24+LPH, 25+LPH, 26+LPH, 27+LPH, 28+LPH, 29+LPH, 30+LPH, 31+LPH, 32+LPH, 33+LPH, 68+LPH, 69+LPH,

    // left under glow - 30
    34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, //67,

    // right underglow - 32
    34+LPH, 35+LPH, 36+LPH, 37+LPH, 38+LPH, 39+LPH, 40+LPH, 41+LPH, 42+LPH, 43+LPH, 44+LPH, 45+LPH, 46+LPH, 47+LPH, 48+LPH, 49+LPH, 50+LPH, 51+LPH, 52+LPH, 53+LPH, 54+LPH, 55+LPH, 56+LPH, 57+LPH, 58+LPH, 59+LPH, 60+LPH, 61+LPH, 62+LPH, 63+LPH, 64+LPH, 65+LPH //, 67+LPH
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

  delay(500);
  enableScannerPower();
  delay(200); // wait for sides bootloader to finish

  // Consider not doing this until 30s after keyboard
  // boot up, to make it easier to rescue things
  // in case of power draw issues.
  leftHandState.all = 0;
  rightHandState.all = 0;

  // initialise Wire of scanner - have to do this here to avoid problem with static object intialisation ordering
  twi_init();

  // load stored keyscanner interval into settings
  settings_base_ = ::EEPROMSettings.requestSlice(sizeof(settings));
  uint8_t keyscan = EEPROM.read(settings_base_);
  if (keyscan == 0xff) {
      EEPROM.update(settings_base_, settings.keyscan);
      EEPROM.commit();
  }
    
  settings.keyscan = EEPROM.read(settings_base_);
  // update left and right side with stored keyscan interval
  leftHand.setKeyscanInterval(settings.keyscan);
  rightHand.setKeyscanInterval(settings.keyscan);

}


// i is number from 0 -> LED_COUNT - 1
void Raise::setCrgbAt(uint8_t i, cRGB crgb) {

  // prevent reading off the end of the led_map array
  if(i >= LED_COUNT)
    return;

  // get the SLED index
  uint8_t sled_num = led_map[i];
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
  setCrgbAt(key_led_map[row][col], color);
}

// returns keyboard LED index
uint8_t Raise::getLedIndex(byte row, byte col) {
  return key_led_map[row][col];
}

// i is number from 0 -> LED_COUNT - 1
// returns LED colour given the keyboard LED index
cRGB Raise::getCrgbAt(uint8_t i) {
  // prevent reading off the end of the led_map array
  if(i >= LED_COUNT)
    return {0, 0, 0};

  uint8_t sled_num = led_map[i];
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

uint16_t Raise::readJoint(){
  return rightHand.readJoint();
}

bool Raise::focusHook(const char *command) {
  enum {
    SIDE_VER,
    SLED_VER,
    ANSI_ISO,
    KEYSCAN,
    JOINT,
    CC,
  } subCommand;

  if (strncmp_P(command, PSTR("hardware."), 9) != 0)
    return false;
  if (strcmp_P(command + 9, PSTR("side_ver")) == 0)
    subCommand = SIDE_VER;
  else if (strcmp_P(command + 9, PSTR("keyscan")) == 0)
    subCommand = KEYSCAN;
  else if (strcmp_P(command + 9, PSTR("sled_ver")) == 0)
    subCommand = SLED_VER;
  else if (strcmp_P(command + 9, PSTR("ansi_iso")) == 0)
    subCommand = ANSI_ISO;
  else if (strcmp_P(command + 9, PSTR("joint")) == 0)
    subCommand = JOINT;
  else if (strcmp_P(command + 9, PSTR("cc")) == 0)
    subCommand = CC;
  else
    return false;

  switch (subCommand) {
  case SLED_VER:
      SerialUSB.print("left: ");
      SerialUSB.println(leftHand.readSLEDVersion());
      SerialUSB.print("right: ");
      SerialUSB.println(rightHand.readSLEDVersion());
    break;
  case SIDE_VER:
      SerialUSB.print("left: ");
      SerialUSB.println(leftHand.readVersion());
      SerialUSB.print("right: ");
      SerialUSB.println(rightHand.readVersion());
    break;
  case ANSI_ISO:
      SerialUSB.print("left: ");
      SerialUSB.println(leftHand.readANSI_ISO() == ANSI ? "ANSI" : "ISO");
      SerialUSB.print("right: ");
      SerialUSB.println(rightHand.readANSI_ISO() == ANSI ? "ANSI": "ISO");
    break;
  case JOINT:
      SerialUSB.println(rightHand.readJoint());
    break;
  case CC:
      SerialUSB.print("UFP :");
      SerialUSB.println(analogRead(UFP_CC));
      SerialUSB.print("DFPL:");
      SerialUSB.println(analogRead(DFPL_CC));
      SerialUSB.print("DFPR:");
      SerialUSB.println(analogRead(DFPR_CC));
    break;
  case KEYSCAN:
    if (SerialUSB.peek() == '\n') {
      SerialUSB.print("left: ");
      SerialUSB.println(leftHand.readKeyscanInterval());
      SerialUSB.print("right: ");
      SerialUSB.println(rightHand.readKeyscanInterval());
    } else {
      settings.keyscan = SerialUSB.parseInt();
      EEPROM.update(settings_base_, settings.keyscan);
      EEPROM.commit();
      leftHand.setKeyscanInterval(settings.keyscan);
      rightHand.setKeyscanInterval(settings.keyscan);
    }
    break;
  }


  return true;
}


HARDWARE_IMPLEMENTATION KeyboardHardware;
