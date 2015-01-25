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

#include "Display.h"

namespace DisplayGroup {

Display::Display() {
  _digits =  const_cast<byte*>(DisplayManager::DEF_DIGITS);
  _bitOrder = DisplayManager::DEF_ORDER;
}

Display::Display(const byte digits[]) {
  _digits = const_cast<byte*>(digits);
  _bitOrder = DisplayManager::DEF_ORDER;
}

Display::~Display() {
}

void Display::update(byte digit) const {
  byte v = _digits[digit];

  for (byte bitMask = 128; bitMask > 0; bitMask >>= 1) {
    digitalWrite(DisplayManager::clockPin, LOW);
    digitalWrite(DisplayManager::dataPin, v & bitMask ? HIGH : LOW);
    digitalWrite(DisplayManager::clockPin, HIGH);
  }
}

void Display::turnOff() const {
  for (byte bitMask = 128; bitMask > 0; bitMask >>= 1) {
    digitalWrite(DisplayManager::clockPin, LOW);
    digitalWrite(DisplayManager::dataPin,  LOW);
    digitalWrite(DisplayManager::clockPin, HIGH);
  }
}

byte Display::getBitOrder() const {
  return _bitOrder;
}

void Display::setBitOrder(byte bitOrder) {
  _bitOrder = bitOrder;
}

} /* namespace DisplayGroup */

