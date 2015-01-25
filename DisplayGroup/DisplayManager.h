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

#ifndef DISPLAYMANAGER_H_
#define DISPLAYMANAGER_H_

// Default pins
#define PIN_COM_DATA 2      // This pin is used to pass the next bit
#define PIN_COM_CLOCK 4     // This pin is used by ShiftOut to clock the data
#define PIN_OUTPUT_ENABLE 3 // This pin gets sets low to enable shift register output

#include <Arduino.h>

#include <DisplayGroup.h>

#include <functional>
#include <iterator>
#include <deque>
#include <algorithm>

namespace DisplayGroup {

class DisplayGroup;

/**
 * @brief Manager for display groups. Add, remove, print and update all the displays at once.
 *
 * @version 1.1
 *
 * This class provides an easy way to manage many 7-segments display in an Arduino application.
 * Each display is connected to a shift register, which holds the data until the next shift is
 * carried out (see http://code.google.com/p/display-group/ for documentation).
 * This class holds an array of groups, which in turn is an array of display, and update the
 * whole array when requested. The update executes in reverse order, to account for the serial
 * hardware link between the shift registers.
 * The update can be done with latch pin low, and after the shift the latch pin is taken high,
 * or with high output enable and transition to low on update. This is configurable through
 * outputEnableState parameter on the constructor.
 * LOW works with typical 74HC595 shift register.
 *
 * This class uses the STL library for Arduino, which can be found at
 * @htmlonly
 * <a href="http://andybrown.me.uk/wk/2011/01/15/the-standard-template-library-stl-for-avr-with-c-streams/">
 * the-standard-template-library-stl-for-avr-with-c-streams </a>
 * @endhtmlonly
 *
 * @author Gionata Boccalini
 * @date   May 13, 2014
 */
class DisplayManager {
public:

  static const byte DEF_DIGITS[10];                 /**< Default digits code for small common cathode 7-segments display */
  static const byte DEF_ORDER;                      /**< Default bit order for the shift */
  static const byte DEF_OUTPUT_ENABLE_W_STATE;      /**< Default logical state (HIGH or LOW) of the output enable (or latch) pin during
                                                         shift register update */

  static byte dataPin;                              /**< Default data output pin */
  static byte clockPin;                             /**< Default clock output pin */
  static byte outputEnablePin;                      /**< Default "output enable" output pin */
  static byte outputEnablePinState;                 /**< Logical state (HIGH or LOW) of the output enable to select a shift register */

  /**
   * Constructor.
   *
   * @param[in] dataP			    Arduino data pin for shift register
   * @param[in] clockP			    Arduino clock pin for shift register
   * @param[in] outputEnableP  	    Arduino output enable pin for shift register
   * @param[in] outputEnableState  	Logical state (HIGH or LOW) of the output enable (or latch) pin during
   *                                shift register update
   */
  DisplayManager(byte dataP, byte clockP, byte outputEnableP, byte outputEnableState);

  /** Default destructor.
   */
  virtual ~DisplayManager();

  /**
   * Add a display group at the end of the data container. The variable pointed to
   * by value is used during the update of the display. Uses default digits code.
   * @param[in] id          Unique Id of the group
   * @param[in] nDisplay    Number of display contained in the group.
   * @param[in] value       Address of the variable to watch
   */
  void addGroup(byte id, byte nDisplay,uint16_t * value);

  /**
   * Add a display group at the end of the data container. The variable pointed to
   * by value is used during the update of the display.
   * @param[in] id          Unique Id of the group
   * @param[in] nDisplay    Number of display contained in the group.
   * @param[in] value       Address of the variable to watch
   * @param[in] digits[]    Array of segment code to initialize all the display in
   *                        the group.
   * @param[in] sizeOfDigits Size of the digits code display
   */
  void addGroup(byte id, byte nDisplay, uint16_t * value, const byte digits[], byte sizeOfDigits);

  /**
   * Add group to the DisplayManager. The group is inserted in the correct order,
   * given the index. The variable pointed to by value is used during the update
   * of the display. Uses default digits code.
   * @param[in] id          Unique Id of the group
   * @param[in] nDisplay	Number of display contained in the group
   * @param[in] index		Index for the group in the application
   * @param[in] value		Address of the variable to watch
   */
  void insertGroup(byte id, byte nDisplay, byte index, uint16_t * value);

  /**
   * Add group to the DisplayManager. The group is inserted in the correct order,
   * given the index. The variable pointed to by value is used during the update
   * of the display.
   * @param[in] id          Unique Id of the group
   * @param[in] nDisplay    Number of display contained in the group
   * @param[in] index       Index for the group in the application
   * @param[in] value       Address of the variable to watch
   * @param[in] digits[]    Array of segment code to initialize all the display in
   *                        the group.
   * @param[in] sizeOfDigits Size of the digits code display
   */
  void insertGroup(byte id, byte nDisplay, byte index, uint16_t * value, const byte digits[], byte sizeOfDigits);


  /**
   * Replace a group with the one built from the given parameters. The group is
   * inserted in the correct order, given the index. The variable pointed to by
   * value is used during the update of the display.
   * @param[in] id          Unique Id of the group to be replaced
   * @param[in] nDisplay    Number of display contained in the group
   * @param[in] value       Address of the variable to watch
   */
  void replaceGroup(byte id, byte nDisplay, uint16_t * value);

  /**
   * Replace a group with the one built from the given parameters. The group is
   * inserted in the correct order, given the index. The variable pointed to by
   * value is used during the update of the display.
   * @param[in] id          Unique Id of the group to be replaced
   * @param[in] nDisplay    Number of display contained in the group
   * @param[in] value       Address of the variable to watch
   * @param[in] digits      Array of segment code to initialize all the display in
   *                        the group.
   * @param[in] sizeOfDigits Size of the digits code display
   */
  void replaceGroup(byte id, byte nDisplay, uint16_t * value, const byte digits[], byte sizeOfDigits);

  /**
   * Remove a group from the manager
   * @param[in] id		    Unique Id of the group
   */
  void removeGroup(byte id);

  /**
   * Clear all the groups from the manager
   */
  void clearGroups();

  /**
   * Enable or disable a group, i.e. a binary zero will be transferred to the displays,
   * turning off all the segments. The displays, or the integrated logic could still be
   * on and powered.
   * @param[in] id          Unique Id of the group
   * @param[in] enable      True to enable the group or false to disable it. All the
   *                        segments will be off.
   */
  void enableGroup(byte id, boolean enable);

  /**
   * Update all the display group in the manager.
   * @return The index of the DisplayGroup with a failure in the update.
   */
  uint16_t updateAll() const;

  /**
   * Sets the update byte order in the group given by index.
   * @param[in] id		   Unique Id of the group
   * @param[in] order	   Bit order: LSBFIRST or MSBFIRST.
   */
  void setBitOrder(byte id, byte order);

  /**
   * Prints the vector of groups in a string object.
   * @return The string representation of the vector of groups.
   */
  String printGroups() const;

private:

  std::deque<DisplayGroup> _groups; /**< Deque of display group */

};

} /* namespace DisplayGroup */

#endif /* DISPLAYMANAGER_H_ */
