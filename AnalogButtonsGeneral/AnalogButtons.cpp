#include "AnalogButtons.h"

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

Button::Button() {
  id = 0;
  BUTTON_L = 0;
  BUTTON_H = AnalogButtons::MAX_ANALOG_V;
  duration = 0;
  isHeldDown = false;
}

Button::Button(int iid, int analogLowVal, int analogHighVal, int holdDuration) {
  id = iid;
  BUTTON_L = analogLowVal;
  BUTTON_H = analogHighVal;
  duration = holdDuration * 1000;
  isHeldDown = false;
}

AnalogButtons::AnalogButtons(int ddebounce_count, void (*pt2Func)(int, boolean)) {
  pt2Function = pt2Func;
  debounce_count = ddebounce_count;
  counter = 0;
  buttonsIndex = 0;
  lastButtonPressed = -1;
  previousMillis = 0;
}

int AnalogButtons::addButton(Button b) {
  if (buttonsIndex < MAXBUTTONS) {
    buttons[buttonsIndex] = b;
    buttonsIndex++;
    return 0;
  } else return -1;
}

void AnalogButtons::checkValue(int analogV) {

    // If no buttons were pressed, reset lastButtonPressed
    boolean foundOne = false;

    for (int i = 0; i < buttonsIndex; i++) {
      if (analogV <= buttons[i].BUTTON_H + 4 && analogV >= buttons[i].BUTTON_L - 4) {
        foundOne = true;

        // First checking for button held down, only if duration is set by user
        if (buttons[i].duration > 0) {
          if (buttons[i].isHeldDown != true && lastButtonPressed == buttons[i].id
              && ((millis() - previousMillis) > buttons[i].duration)) {
            buttons[i].isHeldDown = true;
            buttons[i].held();
            (*pt2Function)(buttons[i].id, true);
            return;
          }
        }

        // Now if a different button has been pressed
        if (lastButtonPressed != buttons[i].id) {
          counter++;
          if (counter >= debounce_count) {
            counter = 0;
            lastButtonPressed = buttons[i].id;
            previousMillis = millis();                  // for detecting held
            buttons[i].pressed();
            (*pt2Function)(buttons[i].id, false);
            return;
          }
        }
      } else {
        // This button was not active so it can't be held down any longer
        buttons[i].isHeldDown = false;
      }
    }

    if (!foundOne) {
      lastButtonPressed = -1;
      counter = 0;
    }
}
