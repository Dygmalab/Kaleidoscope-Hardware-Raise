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
  if (::Focus.handleHelp(command, PSTR("hardware.side_ver\nhardware.sled_ver\nhardware.sled_current\nhardware.ansi_iso\nhardware.joint\nhardware.keyscan")))
    return EventHandlerResult::OK;

  if (strncmp_P(command, PSTR("hardware."), 9) != 0)
    return EventHandlerResult::OK;

  if (strcmp_P(command + 9, PSTR("side_ver")) == 0) {
      ::Focus.send("left: ");
      ::Focus.send(KeyboardHardware.leftVersion());
      ::Focus.send("\nright: ");
      ::Focus.send(KeyboardHardware.rightVersion());
      ::Focus.send("\n");
    return EventHandlerResult::EVENT_CONSUMED;
    }

  if (strcmp_P(command + 9, PSTR("sled_ver")) == 0) {
      ::Focus.send("left: ");
      ::Focus.send(KeyboardHardware.leftSLEDVersion());
      ::Focus.send("\nright: ");
      ::Focus.send(KeyboardHardware.rightSLEDVersion());
      ::Focus.send("\n");
    return EventHandlerResult::EVENT_CONSUMED;
    }

  if (strcmp_P(command + 9, PSTR("sled_current")) == 0)
    if (::Focus.isEOL()) {
      ::Focus.send("left: ");
      ::Focus.send(KeyboardHardware.readLeftSLEDCurrent());
      ::Focus.send("\nright: ");
      ::Focus.send(KeyboardHardware.readRightSLEDCurrent());
      ::Focus.send("\n");
      return EventHandlerResult::EVENT_CONSUMED;
    } else {
      uint8_t current;
      ::Focus.read(current);
      KeyboardHardware.setLeftSLEDCurrent(current);
      KeyboardHardware.setRightSLEDCurrent(current);
      return EventHandlerResult::EVENT_CONSUMED;
      }

  if (strcmp_P(command + 9, PSTR("ansi_iso")) == 0) {
      ::Focus.send("left: ");
      ::Focus.send(KeyboardHardware.readLeftANSI_ISO() == ANSI ? "ANSI" : "ISO");
      ::Focus.send("\nright: ");
      ::Focus.send(KeyboardHardware.readRightANSI_ISO() == ANSI ? "ANSI": "ISO");
      ::Focus.send("\n");
      return EventHandlerResult::EVENT_CONSUMED;
      }

  if (strcmp_P(command + 9, PSTR("joint")) == 0) {
      ::Focus.send(KeyboardHardware.readJoint());
      return EventHandlerResult::EVENT_CONSUMED;
  }
  
  if (strcmp_P(command + 9, PSTR("keyscan")) == 0)
    if (::Focus.isEOL()) {
      ::Focus.send("left: ");
      ::Focus.send(KeyboardHardware.readLeftKeyscanInterval());
      ::Focus.send("\nright: ");
      ::Focus.send(KeyboardHardware.readRightKeyscanInterval());
      ::Focus.send("\n");
      return EventHandlerResult::EVENT_CONSUMED;
    } else {
      uint8_t keyscan;
      ::Focus.read(keyscan);
      // settings.keyscan = keyscan;
      // EEPROM.update(settings_base_, settings.keyscan);
      // EEPROM.commit();
      KeyboardHardware.setLeftKeyscanInterval(keyscan);
      KeyboardHardware.setRightKeyscanInterval(keyscan);
      return EventHandlerResult::EVENT_CONSUMED;
  }

}

}
}

kaleidoscope::plugin::RaiseFocus RaiseFocus;