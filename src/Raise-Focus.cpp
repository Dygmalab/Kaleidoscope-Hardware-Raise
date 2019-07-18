/* -*- mode: c++ -*-
 * Raise Focus
 * Copyright (C) 2019 DygmaLab SE
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <Raise-Focus.h>
#include <Kaleidoscope-FocusSerial.h>

namespace kaleidoscope {
namespace plugin {


EventHandlerResult RaiseFocus::onFocusEvent(const char *command) {
  if (::Focus.handleHelp(command, PSTR("hardware.version\nhardware.side_power\nhardware.side_ver\nhardware.sled_ver\nhardware.sled_current\nhardware.layout\nhardware.joint\nhardware.keyscan\nhardware.crc_errors\nhardware.flash_left_side\nhardware.flash_right_side\nhardware.verify_left_side\nhardware.verify_right_side")))
    return EventHandlerResult::OK;

  if (strncmp_P(command, PSTR("hardware."), 9) != 0)
    return EventHandlerResult::OK;

  if (strcmp_P(command + 9, PSTR("version")) == 0) {
      ::Focus.send("Dygma Raise");
      return EventHandlerResult::EVENT_CONSUMED;
  }

  if (strcmp_P(command + 9, PSTR("flash_left_side")) == 0) {
      ::Focus.send(KeyboardHardware.flashLeftSide());
      return EventHandlerResult::EVENT_CONSUMED;
  }

  if (strcmp_P(command + 9, PSTR("flash_right_side")) == 0) {
      ::Focus.send(KeyboardHardware.flashRightSide());
      return EventHandlerResult::EVENT_CONSUMED;
  }

  if (strcmp_P(command + 9, PSTR("verify_left_side")) == 0) {
      ::Focus.send(KeyboardHardware.verifyLeftSide());
      return EventHandlerResult::EVENT_CONSUMED;
  }

  if (strcmp_P(command + 9, PSTR("verify_right_side")) == 0) {
      ::Focus.send(KeyboardHardware.verifyRightSide());
      return EventHandlerResult::EVENT_CONSUMED;
  }


  if (strcmp_P(command + 9, PSTR("side_power")) == 0)
    if (::Focus.isEOL()) {
      ::Focus.send(KeyboardHardware.getSidePower());
      return EventHandlerResult::EVENT_CONSUMED;
    } else {
      uint8_t power;
      ::Focus.read(power);
      KeyboardHardware.setSidePower(power);
      return EventHandlerResult::EVENT_CONSUMED;
      }

  if (strcmp_P(command + 9, PSTR("side_ver")) == 0) {
      ::Focus.send("left:");
      ::Focus.send(KeyboardHardware.leftVersion());
      ::Focus.send("\nright:");
      ::Focus.send(KeyboardHardware.rightVersion());
    return EventHandlerResult::EVENT_CONSUMED;
    }

  if (strcmp_P(command + 9, PSTR("crc_errors")) == 0) {
      ::Focus.send("left:");
      ::Focus.send(KeyboardHardware.leftCRCErrors());
      ::Focus.send("\nright:");
      ::Focus.send(KeyboardHardware.rightCRCErrors());
    return EventHandlerResult::EVENT_CONSUMED;
    }

  if (strcmp_P(command + 9, PSTR("sled_ver")) == 0) {
      ::Focus.send("left:");
      ::Focus.send(KeyboardHardware.leftSLEDVersion());
      ::Focus.send("\nright:");
      ::Focus.send(KeyboardHardware.rightSLEDVersion());
    return EventHandlerResult::EVENT_CONSUMED;
    }

  if (strcmp_P(command + 9, PSTR("sled_current")) == 0)
    if (::Focus.isEOL()) {
      ::Focus.send("left:");
      ::Focus.send(KeyboardHardware.leftSLEDCurrent());
      ::Focus.send("\nright:");
      ::Focus.send(KeyboardHardware.rightSLEDCurrent());
      return EventHandlerResult::EVENT_CONSUMED;
    } else {
      uint8_t current;
      ::Focus.read(current);
      KeyboardHardware.setSLEDCurrent(current);
      return EventHandlerResult::EVENT_CONSUMED;
      }

  if (strcmp_P(command + 9, PSTR("layout")) == 0) {
      ::Focus.send(KeyboardHardware.readANSI_ISO() == ANSI ? "ANSI" : "ISO");
      return EventHandlerResult::EVENT_CONSUMED;
      }

  if (strcmp_P(command + 9, PSTR("joint")) == 0) {
      ::Focus.send(KeyboardHardware.readJoint());
      return EventHandlerResult::EVENT_CONSUMED;
  }
  
  if (strcmp_P(command + 9, PSTR("keyscan")) == 0)
    if (::Focus.isEOL()) {
      ::Focus.send("left:");
      ::Focus.send(KeyboardHardware.leftKeyscanInterval());
      ::Focus.send("\nright:");
      ::Focus.send(KeyboardHardware.rightKeyscanInterval());
      return EventHandlerResult::EVENT_CONSUMED;
    } else {
      uint8_t keyscan;
      ::Focus.read(keyscan);
      KeyboardHardware.setKeyscanInterval(keyscan);
      return EventHandlerResult::EVENT_CONSUMED;
  }

}

}
}

kaleidoscope::plugin::RaiseFocus RaiseFocus;
