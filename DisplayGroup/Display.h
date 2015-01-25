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

#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <Arduino.h>

#include <DisplayManager.h>

namespace DisplayGroup {

/**
 * @brief Single 7-segment display.
 *
 * This class holds the representation of any digits within the single
 * 7-segment display. The class shifts out the data pin and manages
 * the clock pin to actually display the digits.
 * This class behave like an observer in the "Observer" GoF Design Pattern:
 * it observes the DisplayGroup to which belong and when its Display::update
 * method gets called the actual value to display is converted in binary form
 * and shifted out to the data output pin.
 *
 * The shifting pattern follows the requirements for 74HC595 and 74HC164
 * shift register, which can be found in the datasheet.
 *
 * @author Gionata Boccalini
 * @date   Feb 10, 2013
 */
class Display {
public:

  /**
   * Default Constructor
   */
  Display();

  /**
   * Constructor
   *
   * @param[in] digits	    Array of digits code for every digits [0-9]
   */
  Display(const byte digits[]);

  /** Default Destructor
   */
  virtual ~Display();

  /**
   * Shift out the binary value of update::digit using C bit masking
   *
   * @param[in] digit	    The digits to be displayed
   */
  void update(byte digit) const;

  /**
   * Shift out a binary zero (0) to turn off all the segments.
   */
  void turnOff() const;

  /**
   * @return the byte order of visualization: MSBFIRST or LSBFIRST
   */
  byte getBitOrder() const;

  /**
   * Sets the byte order of visualization: MSBFIRST or LSBFIRST
   *
   * @param[in] bitOrder	The byte order
   */
  void setBitOrder(byte bitOrder);

private:
  byte * _digits; 	/**< arrary of digits codes */
  byte   _bitOrder; /**< byte order of visualization: MSBFIRST (most significant
                         bit first) or LSBFIRST (least significant bit first)) */
};

} /* namespace DisplayGroup */

#endif /* DISPLAY_H_ */
