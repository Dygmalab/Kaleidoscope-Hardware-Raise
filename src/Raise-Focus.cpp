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
  if (::Focus.handleHelp(command, PSTR("hardware.version\nhardware.side_ver\nhardware.sled_ver\nhardware.sled_current\nhardware.ansi_iso\nhardware.joint\nhardware.keyscan")))
    return EventHandlerResult::OK;

  if (strncmp_P(command, PSTR("hardware."), 9) != 0)
    return EventHandlerResult::OK;

  if (strcmp_P(command + 9, PSTR("version")) == 0) {
      ::Focus.send("Dygma Raise");
  }

  if (strcmp_P(command + 9, PSTR("side_ver")) == 0) {
      ::Focus.send("left:");
      ::Focus.send(KeyboardHardware.leftVersion());
      ::Focus.send("\nright:");
      ::Focus.send(KeyboardHardware.rightVersion());
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
      ::Focus.send(KeyboardHardware.readLeftSLEDCurrent());
      ::Focus.send("\nright:");
      ::Focus.send(KeyboardHardware.readRightSLEDCurrent());
      return EventHandlerResult::EVENT_CONSUMED;
    } else {
      uint8_t current;
      ::Focus.read(current);
      KeyboardHardware.setSLEDCurrent(current);
      return EventHandlerResult::EVENT_CONSUMED;
      }

  if (strcmp_P(command + 9, PSTR("ansi_iso")) == 0) {
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
      ::Focus.send(KeyboardHardware.readLeftKeyscanInterval());
      ::Focus.send("\nright:");
      ::Focus.send(KeyboardHardware.readRightKeyscanInterval());
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
