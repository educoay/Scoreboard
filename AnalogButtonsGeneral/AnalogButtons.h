#ifndef ANALOGBUTTONS_H
#define ANALOGBUTTONS_H

/*
  AnalogButtons,
  
  created 02 Jan 2009 V 0.1
 
 Connect more than one button to a single analog pin, 
 register a call-back function.which gets called when a button
 is pressed or held down for the defined number of seconds. Includes
 software key debouncing which may need to be adjusted, the the second 
 argument to AnalogButtons class. Define the ANALOG_PIN in the constructor
 of AnalogButtons.
 
 The circuit:

 * 5 buttons, 1 side of all buttons connected together to +5V. 
   The other side of each button is connected via a different value
   resister (tested with) 1k, 2k5, 5k8, 10k, 18k to one side of a
   100k resister which is in turn connected to GND. At the point
   where all the different resisters are joined you make a connection
   to your analog input. Basicly a different voltage divider is setup 
   depending upon which button is pressed. You have to configure the 
   Buttons Hi/Low values, see the comments in example code below and the
   AnalogButtons::configure(ANALOG_PIN) function.
   
   More or less than 5 buttons could be added, just pick different values
   of the resister sot hat all buttons have different values which arn't too
   close in size.
   
   I'm not sure what happens when Arduino is powered from batteries and Power V
   drops below V5.
 
 by Neil DUdman and everyone who's ever used Arduino
 

 11 May 2014:

   Modification to export the analog value to be read outside of the library,
   into the application code. This is necessary to use, for example, a wireless
   sensor that retreive the readings from a serial connection. So the sensors
   or buttons are NOT physically connected to the Arduino analog input.
 */

#include <Arduino.h>

#define MAXBUTTONS 10

class Button 
{
public:  
  int id;
  int BUTTON_L;
  int BUTTON_H;     

  // For additional functions HOLD
  unsigned long duration;
  boolean isHeldDown;  
  
  Button();
  Button(int iid, int analogLowVal, int analogHighVal, int holdDuration = 0);

  // Override these functions if you want
  void pressed(){ } 

  // Override these functions if you want
  void held(){}
};

class AnalogButtons 
{
private:  
  // For hold button, duration is specified for each button
  unsigned long previousMillis;
  
  // Debouncing of normal pressing
  int debounce_count;
  int counter;
  
  // Status of last press
  int lastButtonPressed;

  Button buttons[MAXBUTTONS];
  int buttonsIndex;
  
  /**
   * Registered Callback function
   *
   * @param[in]     Button ID
   * @param[in]     True if button held
   */
  void (*pt2Function)(int, boolean);    
  
public:

  static const int MAX_ANALOG_V = 1023;         // Maximum analog value that can be read from the ADC
  static const int DEF_DEBOUNCE_COUNT = 100;    // Default button debouncing count

  /**
   * AnalogButtons defaul constructor
   *
   * @param[in] ddebounce_count     Button debouncing count
   * @param[in] *pt2Func            Callback function pointer
   */
  AnalogButtons(int ddebounce_count = DEF_DEBOUNCE_COUNT, void (*pt2Func)(int, boolean) = 0);
  
  /**
   * Adds a button object to this analog input
   *
   * @param[in] b                   Button instance
   */
  int addButton(Button b);
  
  /**
   * Checks the current input value
   *
   * @param[in] analogV             analog input value
   */
  void checkValue(int analogV);
  
  /**
   * Debug helper: press each button in turn and note the values
   * returned to serial monitor
   */
  static void configure(int pin);
};

#endif
