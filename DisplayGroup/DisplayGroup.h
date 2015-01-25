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

#ifndef DISPLAYGROUP_H_
#define DISPLAYGROUP_H_

#include <Arduino.h>

#include <Display.h>

#include <iterator>
#include <vector>

namespace DisplayGroup {

class Display;

/**
 * @brief Group of 7-segments displays. Holds a vector of Display objects, updates every display
 * with the correct digit.
 *
 * This class represents a group of n 7-segments displays, which can show an n digits number.
 * Each group has a vector of displays, and a value to show, using all the displays.
 * The class take the address of the variable to monitor, and does all the computation to send
 * the i-th digit to the i-th display, with automatic zero padding on the heading and trailing
 * if the value to show uses less digits than the number of displays in the group.
 * If the value to show uses more digits than the number of displays in the group only the least
 * significant digits are showed.
 *
 * This class uses the STL library for Arduino, which can be found at
 * @htmlonly
 * <a href="http://andybrown.me.uk/wk/2011/01/15/the-standard-template-library-stl-for-avr-with-c-streams/">
 * the-standard-template-library-stl-for-avr-with-c-streams </a>
 * @endhtmlonly
 *
 * @author Gionata Boccalini
 * @date   Feb 10, 2013
 */
class DisplayGroup {
public:

  /**
   * Constructor.
   *
   * @param[in] nDisplay	Number of display in the group
   * @param[in] id	    	Id of the DisplayGroup in the manager
   * @param[in] *value  	Address of the value to be monitored
   * @param[in] digits[]    Array of 7-segments code for each digits [0-9]
   */
  DisplayGroup(byte nDisplay, byte id, uint16_t * value, const byte digits[]);


  /** Default destructor.
   */
  virtual ~DisplayGroup();

  /**
   * Scans the value to be showed and count how many digits must be sent to the displays.
   * The count is made by dividing and considering the remainder of divisions by 10, until
   * the quotient is zero or there are no display available in the group any more.
   * The method does automatic padding (filling) with zeros when needed in the heading and
   * trailing of the displays vector.
   *
   * @return -1		If _nDisplay is equal to zero
   * @return -2		If _value is NULL
   * @return -3		If the whole value cannot be displayed with the number of displays in
   *				the group
   * @return  0		On success
   */
  int update() const;

  /**
   *
   * @return The unique id of the DisplayGroup in the manager
   */
  byte getId() const;

  /**
   *
   * @return The number of display in the group
   */
  byte getDisplayNumber() const;

  /**
   *
   * @return The bit order in every display
   */
  byte getBitOrder() const;

  /**
   *
   * @param[in] bitOrder	The bit order in every display. Can LSBFIRST or MSBFIRST,
   * 						as in Arduino specification
   */
  void setBitOrder(byte bitOrder);

  /**
   * Set the enable flag
   * @param enabled         True to enable the group or false to disable it. All the
   *                        segments will be off.
   */
  void setEnabled(boolean enabled);

private:

  std::vector<Display> _displays; /**< Vector of 7-segments displays */
  byte _id;                       /**< Id of the DisplayGroup */
  uint16_t * _value;              /**< Address of the value to be monitored */
  byte _nDisplay;                 /**< Number of display in the group */
  byte _bitOrder;                 /**< Bit order in every display */
  boolean _enabled;               /**< Enable flag */
};

} /* namespace DisplayGroup */

#endif /* DISPLAYGROUP_H_ */
