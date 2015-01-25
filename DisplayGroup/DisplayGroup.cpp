/*
 *  This file is part of DisplayGroup Library.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *    http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include "DisplayGroup.h"

namespace DisplayGroup {

DisplayGroup::DisplayGroup(byte nDisplay, byte id, uint16_t * value, const byte digits[]) :
      _id(id), _nDisplay(nDisplay) {

  Display dis(digits);
  _displays.assign(nDisplay, dis);

  _value = value;
  _enabled = true;
  _bitOrder = DisplayManager::DEF_ORDER;
}

DisplayGroup::~DisplayGroup() {

}

int DisplayGroup::update() const {
  if (_nDisplay == 0) {
    return -1;
  }

  // Turn off al the displays
  if (!_enabled) {
    std::vector<Display>::const_iterator beg = _displays.begin();
    std::vector<Display>::const_iterator end = _displays.end();

    for (; beg != end; ++beg) {
      (*beg).turnOff();
    }

    return 0;
  }

  if (!_value) {
    return -2;
  }

  // Quotient
  uint16_t quot = *(_value) / 10;
  uint16_t tempV = quot;
  // Division counter
  byte divCount = 1;
  boolean filled = false;

  // Update the first display
  _displays[0].update(*(_value) % 10);

  // Look for zero filling in heading when *(_value) is < 10
  if (quot == 0) {
    for (byte i = 1; i < _nDisplay; ++i) {
      _displays[i].update(0);
    }
    filled = true;
  }

  // Count up division by 10 updates display with the remainder
  while (quot != 0 && divCount < _nDisplay) {
    quot = tempV / 10;

    _displays[divCount].update(tempV % 10);
    divCount++;

    tempV = quot;
  }

  // Return -3 if the number cannot be displayed with the number of displays in
  // the group
  if (divCount == _nDisplay && quot != 0) {
    return -3;
  } else if (divCount < _nDisplay && !filled) {
    // Look for zero filling in heading when *(_value) is > 10
    for (byte i = divCount; i < _nDisplay; ++i) {
      _displays[i].update(0);
    }
  }

  return 0;
}

byte DisplayGroup::getId() const {
  return _id;
}

byte DisplayGroup::getDisplayNumber() const {
  return _nDisplay;
}

byte DisplayGroup::getBitOrder() const {
  return _bitOrder;
}

void DisplayGroup::setBitOrder(byte byteOrder) {
  _bitOrder = byteOrder;

  std::vector<Display>::iterator beg = _displays.begin();
  std::vector<Display>::iterator end = _displays.end();
  for (; beg != end; ++beg) {
    (*beg).setBitOrder(_bitOrder);
  }
}

void DisplayGroup::setEnabled(boolean enabled) {
  _enabled = enabled;
}

} /* namespace DisplayGroup */

