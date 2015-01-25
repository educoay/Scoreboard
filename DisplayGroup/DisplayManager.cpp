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

#include "DisplayManager.h"

namespace DisplayGroup {

const byte DisplayManager::DEF_DIGITS[10] = { 1 + 4 + 8 + 16 + 32 + 64,
                                              16 + 64,
                                              1 + 2 + 8 + 16 + 32,
                                              2 + 8 + 16 + 32 + 64,
                                              2 + 4 + 16 + 64,
                                              2 + 4 + 8 + 32 + 64,
                                              1 + 2 + 4 + 8 + 32 + 64,
                                              8 + 16 + 64,
                                              1 + 2 + 4 + 8 + 16 + 32 + 64,
                                              2 + 4 + 8 + 16 + 32 + 64 };

const byte DisplayManager::DEF_ORDER = LSBFIRST;
const byte DisplayManager::DEF_OUTPUT_ENABLE_W_STATE = HIGH;

byte DisplayManager::dataPin = PIN_COM_DATA;
byte DisplayManager::clockPin = PIN_COM_CLOCK;
byte DisplayManager::outputEnablePin = PIN_OUTPUT_ENABLE;
byte DisplayManager::outputEnablePinState = DEF_OUTPUT_ENABLE_W_STATE;

/**
 * @brief Binary predicate for STL find_if, replace_if algorithms
 */
struct GroupId: public std::binary_function<DisplayGroup, byte, bool> {
  /**
   * Unary operator to compare DisplayGroups
   * @param[in] gr      DisplayGroup in input
   * @param[in] id      Id to match
   * @return
   */
  bool operator () (const DisplayGroup &gr, const byte &id) const {
    return gr.getId() == id;
  }
};

DisplayManager::DisplayManager(byte dataP, byte clockP, byte outputEnableP, byte outputEnableState) {
  // Setup static variables
  dataPin = dataP;
  clockPin = clockP;
  outputEnablePin = outputEnableP;
  outputEnablePinState = outputEnableState;

  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(outputEnablePin, OUTPUT);

  digitalWrite(outputEnablePin, !outputEnablePinState);
}

DisplayManager::~DisplayManager() {

}

void DisplayManager::addGroup(byte id, byte nDisplay, uint16_t * value) {
  addGroup(id, nDisplay, value, DEF_DIGITS, sizeof(DEF_DIGITS));
}

void DisplayManager::addGroup(byte id, byte nDisplay, uint16_t * value, const byte digits[], byte sizeOfDigits) {
  assert(digits != NULL && sizeOfDigits == 10);

  if (std::find_if(_groups.begin(), _groups.end(), std::bind2nd(GroupId(), id)) == _groups.end()) {
    // A group with this id has not been found
    DisplayGroup disGroup(nDisplay, id, value, digits);
    _groups.push_back(disGroup);
  }
}

void DisplayManager::insertGroup(byte id, byte nDisplay, byte index, uint16_t * value) {
  insertGroup(id, nDisplay, index, value, DEF_DIGITS, sizeof(DEF_DIGITS));
}

void DisplayManager::insertGroup(byte id, byte nDisplay, byte index, uint16_t * value, const byte digits[], byte sizeOfDigits) {
  assert(index >= 0 && index <= _groups.size() && digits != NULL && sizeOfDigits == 10);

  if (std::find_if(_groups.begin(), _groups.end(), std::bind2nd(GroupId(), id)) == _groups.end()) {
    // A group with this id has not been found
    DisplayGroup disGroup(nDisplay, id, value, digits);
    _groups.insert(_groups.begin() + index, disGroup);
  }
}

void DisplayManager::replaceGroup(byte id, byte nDisplay, uint16_t * value) {
  replaceGroup(id, nDisplay, value, DEF_DIGITS, sizeof(DEF_DIGITS));
}

void DisplayManager::replaceGroup(byte id, byte nDisplay, uint16_t * value, const byte digits[], byte sizeOfDigits) {
  assert(digits != NULL && sizeOfDigits == 10);

  DisplayGroup disGroup(nDisplay, id, value, digits);
  std::replace_if(_groups.begin(), _groups.end(), std::bind2nd(GroupId(), id), disGroup);
}

void DisplayManager::removeGroup(byte id) {
  std::deque<DisplayGroup>::iterator res = std::find_if(_groups.begin(), _groups.end(), std::bind2nd(GroupId(), id));

  if (res != _groups.end()) {
    _groups.erase(res);
  }
}

void DisplayManager::clearGroups() {
  _groups.clear();
}

void DisplayManager::setBitOrder(byte id, byte order) {
  if (order > MSBFIRST)
    return;

  std::deque<DisplayGroup>::iterator res = std::find_if(_groups.begin(), _groups.end(), std::bind2nd(GroupId(), id));

  if (res != _groups.end()) {
    res->setBitOrder(order);
  }
}

void DisplayManager::enableGroup(byte id, boolean enable){
  std::deque<DisplayGroup>::iterator res = std::find_if(_groups.begin(), _groups.end(), std::bind2nd(GroupId(), id));

  if (res != _groups.end()) {
    res->setEnabled(enable);
  }
}

uint16_t DisplayManager::updateAll() const {
  uint16_t ret = 0, idx = 0;
  byte outDisable = (outputEnablePinState == HIGH ? LOW : HIGH);

  digitalWrite(outputEnablePin, outputEnablePinState);

  // Reverse iteration to account for shift register serial update order
  std::deque<DisplayGroup>::const_reverse_iterator beg = _groups.rbegin();
  std::deque<DisplayGroup>::const_reverse_iterator end = _groups.rend();

  for (idx = 0; beg != end; ++beg, ++idx) {
    if ((*beg).update() != 0) {
      ret = idx;
    }
  }

  digitalWrite(outputEnablePin, outDisable);

  return ret;
}

String DisplayManager::printGroups() const {
  String out = "";
  std::deque<DisplayGroup>::const_iterator beg = _groups.begin();
  std::deque<DisplayGroup>::const_iterator end = _groups.end();

  uint16_t i = 0;

  for (; beg != end; ++beg, ++i) {
    out += "Group idx = " + String(i) +  ", # display = " + (*beg).getDisplayNumber() + "\n";
  }
  return out;
}

} /* namespace DisplayGroup */

