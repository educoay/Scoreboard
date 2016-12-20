/*
 *  This file is part of ScoreBoard project.
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

/*
 * ScoreBoard project main file.
 * version 1.0.5
 */

#include <Arduino.h>
#include <avr/eeprom.h>

#include <pnew.cpp>

#include <DisplayManager.h>
#include <AnalogButtons.h>

// Analog input buttons IDs
#define TIMER_ANALOG_INPUT      5
#define AWAY_ANALOG_INPUT       3
#define HOME_ANALOG_INPUT       1

// Digital outputs
#define BUZZER_OUTPUT           8
#define PIN_COM_DATA            2             // Data output pin: used to pass the next bit
#define PIN_COM_CLOCK           4             // Clock output pin: used by DisplayManager to clock the data
#define PIN_OUTPUT_ENABLE       3             // Output enable pin: set to low to enable shift register output

// Home buttons IDs
#define HOME_P1                 1
#define HOME_P2                 2
#define HOME_P3                 3
#define HOME_M1                 4

// Away buttons IDs
#define AWAY_P1                 5
#define AWAY_P2                 6
#define AWAY_P3                 7
#define AWAY_M1                 8

// Timer and setup buttons IDs
#define TIMER_START_STOP        9
#define TIMER_RESET             10
#define PERIOD_P1               11
#define SETUP_MODE              12

// Button held time
#define HELD_DURATION           3

// Update display interval when idle in ms
#define UPDATE_DISPLAY_INT      500

// Period end horn/buzzer on time in ms
#define BUZZER_ON_TIME          900
#define BUZZER_ON_END_TIME      1300

// Button debouncing and timer count
#define DEBOUNCING_COUNT        75
#define DEBOUNCING_COUNT_HOME   80
#define MIN_VALID_ANALOG_VALUE  15
#define TIMER_INIT_MIN          10L
#define TIMER_INIT_SEC          0L

#define MAX_MINUTES             20

#define EEPROM_MAX_WRITE        100000        // Maximum number of erase-write cycles for EVERY EEPROM cell
#define EEPROM_SIZE             1024          // EEPROM size in bytes

// Timer macros
#define START_STOP_TIMER(sreg,v) \
  sreg = SREG;\
  cli();\
  TCCR1B = v;\
  SREG = sreg;

#define RESET_TIMER_COUNTER(sreg) \
  sreg = SREG;\
  cli();\
  TCNT1 = 0;\
  SREG = sreg;

// ####################### Prototypes #######################

void configureAnalogB();

void setupDiplay();

  // Handle home buttons pressed
void handleHomeButtons(int id, boolean held);

// Handle away buttons pressed and setup mode
void handleAwayButtons(int id, boolean held);

// Handle timer buttons pressed and setup mode
void handleTimerButtons(int id, boolean held);

// EEPROM wear leveling algorithm prototypes
// Seek for the first writable cells in the EEPROM  i.e. the first cell counter
// with value smaller than EEPROM_MAX_WRITE
boolean initializeEEPROM();

// Write to the EEPROM cell with offset "offsetEE" and counter "counterEE" the
// actual value of score and time variables.
boolean writeEEPROM();

// Read from the EEPROM cell with offset "offsetEE" and counter "counterEE" the
// stored value of score and time variables.
void readEEPROM();

// Prints on the serial interface the contents of all the EEPROM cells
void printEEPROM();

// Buzzer managament: end of period or manual activation
void buzzer(boolean activation);


// ######################## Constants ########################

// Analog input values read from digital buttons
#ifdef PROTOTYPE
const uint16_t bAVal[10] = { 65, 100, 125, 160, 240, 300, 340, 385 };
#else
const uint16_t bAVal[10] = { 50, 95, 110, 170, 200, 295, 310, 390 };
#endif

// 7-Segments code for big 7" display
const byte gDigits7[10] = { 1 + 2 + 4 + 8 + 16 + 32,
                            2 + 4 + 128,
                            1 + 2 + 8 + 16 + 64,
                            1 + 2 + 4 + 8 + 64,
                            2 + 4 + 32 + 64 + 128,
                            1 + 4 + 8 + 32 + 64,
                            1 + 4 + 8 + 16 + 32+ 64,
                            1 + 2 + 4,
                            1 + 2 + 4 + 8 + 16 + 32 + 64,
                            1 + 2 + 4 + 8 + 32 + 64 };

// 7-Segments code for 4" display
const byte gDigits4[10] = { 2 + 4 + 8 + 16 + 32 + 64,
                            4 + 8,
                            2 + 4 + 16 + 32 + 128,
                            2 + 4 + 8 + 16 + 128,
                            4 + 8 + 64 + 128,
                            2 + 8 + 16 + 64 + 128,
                            2 + 8 + 16 + 32 + 64 + 128,
                            2 + 4 + 8,
                            2 + 4 + 8 + 16 + 32 + 64 + 128,
                            2 + 4 + 8 + 16 + 64 + 128 };

// ####################### Data types #######################
struct Score {
  uint16_t home;
  uint16_t away;
};

struct Time {
  uint16_t min;
  uint16_t sec;
  uint16_t period;
};

struct Sets {
  uint16_t actSet;
  uint16_t homeSet;
  uint16_t awaySet;
};

// Persistent data type to be written on the EEPROM
struct persistentData {
  uint32_t counter;
  Score score;
  Time time;
};

// ####################### Variables #######################
Score bScore;
Time time;

boolean timerRunning = false;
volatile boolean updateDisplay = true;
volatile boolean saveEEprom = false;
boolean setupMode = false;
boolean inputEnable = false;
boolean buzzerFired = false;
boolean buzzerManCmd = false;
unsigned long upDisplayTime = 0;
unsigned long buzzerOnTime = 0;
volatile unsigned long sec = 0;

// Volleyball mode
boolean volleyMode = false;
Score vScore;
Sets vSets;


// EEPROM wear leveling algorithm variables
// Buffer for store values from EEPROM on first read upon startup
persistentData dataEE;

// Counts up the erase-write cycle in the current cell pointed to by offsetEE
uint32_t counterEE;

// Points to the location in EEPROM where it is possible to save data (still
// no end of life cell i.e. counteEE < EEPROM_MAX_WRITE)
uint16_t offsetEE;

// Signal end of EEPROM life
boolean endOfLifeEE = false;

// #########################################################
// ############### 7-segment display library ###############

// Different output enable state for different type of shift register
#ifdef PROTOTYPE
DisplayGroup::DisplayManager disManager(PIN_COM_DATA, PIN_COM_CLOCK, PIN_OUTPUT_ENABLE, LOW);
#else
DisplayGroup::DisplayManager disManager(PIN_COM_DATA, PIN_COM_CLOCK, PIN_OUTPUT_ENABLE, HIGH);
#endif

// #########################################################


// #########################################################
// Configuration of digital buttons on the analog interface

AnalogButtons homeButtons(DEBOUNCING_COUNT_HOME, &handleHomeButtons);
Button b1 = Button(HOME_P1, bAVal[0], bAVal[1]);
Button b2 = Button(HOME_P2, bAVal[2], bAVal[3]);
Button b3 = Button(HOME_P3, bAVal[4], bAVal[5]);
Button b4 = Button(HOME_M1, bAVal[6], bAVal[7]);

AnalogButtons awayButtons(DEBOUNCING_COUNT, &handleAwayButtons);
Button b5 = Button(AWAY_P1, bAVal[0], bAVal[1]);
Button b6 = Button(AWAY_P2, bAVal[2], bAVal[3]);
Button b7 = Button(AWAY_P3, bAVal[4], bAVal[5]);
Button b8 = Button(AWAY_M1, bAVal[6], bAVal[7]);

AnalogButtons timerButtons(DEBOUNCING_COUNT, &handleTimerButtons);
Button b9 = Button(TIMER_START_STOP, bAVal[0], bAVal[1]);
Button b10 = Button(TIMER_RESET, bAVal[2], bAVal[3], HELD_DURATION);
Button b11 = Button(PERIOD_P1, bAVal[4], bAVal[5]);
Button b12 = Button(SETUP_MODE, bAVal[6], bAVal[7], HELD_DURATION);

// #########################################################

void configureAnalogB() {
  AnalogButtons::configure(HOME_ANALOG_INPUT);
  AnalogButtons::configure(AWAY_ANALOG_INPUT);
  AnalogButtons::configure(TIMER_ANALOG_INPUT);
  Serial.println("-------");
  delay(200);
}

void handleHomeButtons(int id, boolean held) {

#ifdef DEBUG
  Serial.print("HOME = ");
  Serial.print(id);
  Serial.print(",  HELD = ");
  Serial.println(held);
#endif

  updateDisplay = true;

  switch (id) {

    case HOME_P1:
      if (setupMode) {
        printEEPROM();
        return;
      }

      if (volleyMode) {
        vScore.home++;
        return;
      }

      // Increment home score
      bScore.home++;
      saveEEprom = true;
      return;

    case HOME_P2:
      if (setupMode) {
        return;
      }

      if (volleyMode) {
        if (vScore.home > 0) {
          vScore.home--;
        }
        return;
      }

      bScore.home += 2;
      saveEEprom = true;
      return;

    case HOME_P3:
      if (setupMode) {
        buzzerManCmd = true;
        return;
      }

      if (volleyMode) {
        if (vSets.homeSet < 3 && vSets.awaySet < 3) {
          vSets.homeSet++;
        }
        return;
      }

      bScore.home += 3;
      saveEEprom = true;
      return;

    case HOME_M1:
      if (setupMode) {

        volleyMode = !volleyMode;

        if (volleyMode) {
		  // Volleyball mode
          disManager.clearGroups();

#ifdef PROTOTYPE
          disManager.addGroup(0, 1, NULL);
          disManager.addGroup(1, 1, &vSets.homeSet);
          disManager.addGroup(2, 1, &vSets.awaySet);
          disManager.addGroup(3, 1, NULL);
          disManager.addGroup(4, 1, &vSets.actSet);
          disManager.addGroup(5, 2, &vScore.home);
          disManager.addGroup(6, 2, &vScore.away);

          disManager.enableGroup(0, false);
          disManager.enableGroup(3, false);
#else
          disManager.addGroup(0, 2, &vScore.home, gDigits7, sizeof(gDigits7));
          disManager.addGroup(1, 2, &vScore.away, gDigits7, sizeof(gDigits7));
          disManager.addGroup(2, 1, &vSets.actSet, gDigits7, sizeof(gDigits7));
          disManager.addGroup(3, 1, NULL);
          disManager.addGroup(4, 1, &vSets.homeSet, gDigits4, sizeof(gDigits4));
          disManager.addGroup(5, 1, &vSets.awaySet, gDigits4, sizeof(gDigits4));
          disManager.addGroup(6, 1, NULL);

          disManager.enableGroup(3, false);
          disManager.enableGroup(6, false);
#endif
        } else {
          // Basketball mode
		  disManager.clearGroups();
          setupDiplay();
        }
        setupMode = false;
        return;
      }

      if (volleyMode) {
        if (vSets.homeSet > 0) {
          vSets.homeSet--;
        }
        return;
      }

      // Decrement basketball home score
      if (bScore.home > 0) {
        bScore.home--;
        saveEEprom = true;
      }
      return;
  }
}

void handleAwayButtons(int id, boolean held) {
  unsigned char sreg;

#ifdef DEBUG
  Serial.print("AWAY = ");
  Serial.print(id);
  Serial.print(",  HELD = ");
  Serial.println(held);
#endif

  if (setupMode && volleyMode) {
    return;
  }

  updateDisplay = true;

  switch (id) {

    case AWAY_P1:
      // Increment away score or minute in setup mode
      if (setupMode && !volleyMode) {
        if (time.min == MAX_MINUTES) {
          time.min = 0;
        } else {
		  time.min++;
		}
		sec = time.min * 60 + time.sec;
		RESET_TIMER_COUNTER(sreg);
        return;
      }

      if (volleyMode) {
        vScore.away++;
        return;
      }

      bScore.away++;
      saveEEprom = true;
      return;

    case AWAY_P2:
      if (setupMode && !volleyMode) {
        time.sec++;
        if (time.sec > 59) {
          time.sec = 0;
        }
	    sec = time.min * 60 + time.sec;
	    RESET_TIMER_COUNTER(sreg);
        return;
      }

      if (volleyMode) {
        if (vScore.away > 0) {
          vScore.away--;
        }
        return;
      }

      bScore.away += 2;
      saveEEprom = true;
      return;

    case AWAY_P3:
      if (setupMode && !volleyMode) {
        if (time.sec == 0) {
          time.sec = 59;
        } else {
		 time.sec--;
		}
		sec = time.min * 60 + time.sec;
		RESET_TIMER_COUNTER(sreg);
        return;
      }

      if (volleyMode) {
        if (vSets.awaySet < 3 && vSets.homeSet < 3) {
          vSets.awaySet++;
        }
        return;
      }

      bScore.away += 3;
      saveEEprom = true;
      return;

    case AWAY_M1:
      if (setupMode && !volleyMode) {
        time.sec += 5;
        time.sec %= 60;
		sec = time.min * 60 + time.sec;
		RESET_TIMER_COUNTER(sreg);
        return;
      }

      if (volleyMode) {
        if (vSets.awaySet > 0) {
          vSets.awaySet--;
        }
        return;
      }

      // Decrement basketball away score
      if (bScore.away > 0) {
        bScore.away--;
        saveEEprom = true;
      }
      return;
  }
}

void handleTimerButtons(int id, boolean held) {
  unsigned char sreg;

#ifdef DEBUG
  Serial.print("TIMER = ");
  Serial.print(id);
  Serial.print(",  HELD = ");
  Serial.println(held);
#endif

  updateDisplay = true;

  switch (id) {

    case TIMER_START_STOP:
      if (setupMode or volleyMode) {
        return;
      }

      if (timerRunning) {
        // Stop timer, save global interrupt flag and restore after
        START_STOP_TIMER(sreg, 0);
        timerRunning = false;
      } else {
        // Start/restart timer
        if (time.min == 0 && time.sec == 0) {
          return;
        }
        // Start timer
        START_STOP_TIMER(sreg, 13);
        timerRunning = true;
      }
      return;

    case TIMER_RESET:
      if (setupMode) {
        if (!volleyMode) {
          // Reset score
          bScore.home = 0;
          bScore.away = 0;
          return;
        } else return;
      }

      if (volleyMode && held) {
        vScore.home = 0;
        vScore.away = 0;
        return;
      }

      if (!timerRunning && held) {
        RESET_TIMER_COUNTER(sreg);

        time.min = TIMER_INIT_MIN;
        time.sec = TIMER_INIT_SEC;
        sec = TIMER_INIT_MIN * 60 + TIMER_INIT_SEC;
      }
      return;

    case PERIOD_P1:
      // If setup mode is active reload values from EEPROM internal memory (basketball score)
      if (setupMode) {
        if (!volleyMode) {
          bScore = dataEE.score;
          time = dataEE.time;
          sec = time.min * 60 + time.sec;
          return;
        } else return;
      }

      if (volleyMode) {
        if (vSets.actSet == 5) {
          vSets.actSet = 1;
        } else vSets.actSet++;
        return;
      }

      if (time.period == 6) {
        time.period = 1;
      } else time.period++;

      saveEEprom = true;
      return;

    case SETUP_MODE:
      // If held activates "setup mode": change of current time and reload
      // values from EEPROM memory
      if (!timerRunning && held) {
        setupMode = true;
      } else if (!timerRunning && setupMode) {
        setupMode = false;
        saveEEprom = true;
      }
      return;
  }
}

ISR(TIMER1_COMPA_vect) {
  sec--;
  updateDisplay = true;
  saveEEprom = true;
}

// #########################################################
// ####### EEPROM wear leveling algorithm functions ########
// #########################################################

void resetEEPROM() {
  persistentData data;

  data.counter = 0;
  data.score.away = 0;
  data.score.home = 0;
  data.time.min = 0;
  data.time.sec = 0;
  data.time.period = 0;

  uint16_t size = sizeof(data);

  uint16_t writeCounter = (uint16_t) EEPROM_SIZE / size;

  for (uint16_t i = 0; i < writeCounter; i++) {
    eeprom_write_block((void*) &data, (void*) (size * i), sizeof(data));
  }
}

boolean initializeEEPROM() {
  persistentData data;

  uint16_t size = sizeof(data);

  uint16_t readCounter = (uint16_t) EEPROM_SIZE / size;

  for (uint16_t i = 0; i < readCounter; i++) {
    eeprom_read_block((void*) &data, (void*) (size * i), sizeof(data));

    if (data.counter < EEPROM_MAX_WRITE) {
      if (data.counter == 0 && i > 0) {
        // Read from previous cell, just reached end of life
        i--;
        eeprom_read_block((void*) &data, (void*) (size * i), sizeof(data));
      }
      counterEE = data.counter;
      offsetEE = size * i;
      dataEE = data;
      return true;
    }
  }

  // EEPROM full (end of life), read last saved data
  dataEE = data;
  return false;
}

boolean writeEEPROM() {
  persistentData data;
  uint16_t size = sizeof(data);

  counterEE++;

  if (counterEE > EEPROM_MAX_WRITE) {
    offsetEE += size;
    counterEE = 1;
  }

  if (offsetEE >= EEPROM_SIZE) {
    return false;
  }

  data.counter = counterEE;
  data.score = bScore;
  data.time = time;

  eeprom_write_block((void*) &data, (void*) offsetEE, sizeof(data));

  return true;
}

void readEEPROM() {
  persistentData data;

  if (offsetEE + sizeof(data) <= EEPROM_SIZE) {
    eeprom_read_block((void*) &data, (void*) offsetEE, sizeof(data));
    bScore = data.score;
    time = data.time;
  }
}

void printEEPROM() {
  persistentData data;

  uint16_t size = sizeof(data);

  uint16_t readCounter = (uint16_t) EEPROM_SIZE / size;

  Serial.println("###############################");
  Serial.println("####### EEPROM CONTENTS #######");
  Serial.print("record count = ");
  Serial.println(readCounter);
  Serial.println();

  for (uint16_t i = 0; i < readCounter; i++) {
    eeprom_read_block((void*) &data, (void*) (size * i), sizeof(data));

    Serial.print("Counter = ");
    Serial.println(data.counter);
    Serial.print("Score = ");
    Serial.print(data.score.home);
    Serial.print(", ");
    Serial.println(data.score.away);
    Serial.print("Time = ");
    Serial.print(data.time.min);
    Serial.print(":");
    Serial.print(data.time.sec);
    Serial.print(", ");
    Serial.println(data.time.period);
    Serial.println();
  }
}


// Horn/buzzer management
void buzzer(boolean activation) {
  unsigned long buzzerTime;

  if (time.period >= 4) {
    buzzerTime = BUZZER_ON_END_TIME;
  } else {
    buzzerTime = BUZZER_ON_TIME;
  }

  if (activation) {
    if (!buzzerFired) {
      buzzerOnTime = millis();
      digitalWrite(BUZZER_OUTPUT, LOW);
      buzzerFired = true;
    }
  }

  if (buzzerFired && millis() - buzzerOnTime > buzzerTime) {
    buzzerFired = false;
    digitalWrite(BUZZER_OUTPUT, HIGH);
  }
}


// ========================================================
// |                        SETUP                         |
// ========================================================

void setupDiplay() {
#ifdef PROTOTYPE
    // Prototype breadboard display
    disManager.addGroup(0, 2, &time.min);
    disManager.addGroup(1, 2, &time.sec);
    disManager.addGroup(2, 1, &time.period);
    disManager.addGroup(3, 2, &bScore.home);
    disManager.addGroup(4, 2, &bScore.away);
#else
    // Big display
    disManager.addGroup(0, 2, &bScore.home, gDigits7, sizeof(gDigits7));
    disManager.addGroup(1, 2, &bScore.away, gDigits7, sizeof(gDigits7));
    disManager.addGroup(2, 1, &time.period, gDigits7, sizeof(gDigits7));
    disManager.addGroup(3, 2, &time.min, gDigits4, sizeof(gDigits4));
    disManager.addGroup(4, 2, &time.sec, gDigits4, sizeof(gDigits4));
#endif
}

void setup() {
  pinMode(BUZZER_OUTPUT, OUTPUT);

  digitalWrite(BUZZER_OUTPUT, HIGH);

  Serial.begin(115200);

  // Display manager setup
  setupDiplay();

  // Input Buttons
  homeButtons.addButton(b1);
  homeButtons.addButton(b2);
  homeButtons.addButton(b3);
  homeButtons.addButton(b4);

  awayButtons.addButton(b5);
  awayButtons.addButton(b6);
  awayButtons.addButton(b7);
  awayButtons.addButton(b8);

  timerButtons.addButton(b9);
  timerButtons.addButton(b10);
  timerButtons.addButton(b11);
  timerButtons.addButton(b12);

  // Variable initialization
  bScore.home = 0;
  bScore.away = 0;

  time.period = 1;
  time.min = TIMER_INIT_MIN;
  time.sec = TIMER_INIT_SEC;
  sec = TIMER_INIT_MIN * 60 + TIMER_INIT_SEC;

  vScore.home = 0;
  vScore.away = 0;

  vSets.actSet = 1;
  vSets.homeSet = 0;
  vSets.awaySet = 0;

  timerRunning = false;

  // Setup of timer1 CTC interrupt
  // Initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B

  // Set compare match register to desired timer count
  OCR1A = 15624;
  // Turn on CTC mode
  // TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  // TCCR1B |= (1 << CS10);
  // TCCR1B |= (1 << CS12);
  // TCCR1B = 13;

  // Enable timer compare interrupt:
  TIMSK1 |= _BV(OCIE1A);
  TCNT1 = 0;                // reset counter
  TIFR1 |= _BV(OCF1A);      // clear interrupt flag
  sei();                    // Enable global interrupts

  disManager.updateAll();

  counterEE = 0;
  offsetEE = EEPROM_SIZE;

  endOfLifeEE = !initializeEEPROM();

  if (endOfLifeEE) {
    for (int i = 0; i < 20; ++i) {
      digitalWrite(PIN_OUTPUT_ENABLE, (i % 2 == 0));
      delay(500);
    }
  }
} // End of setup

// ========================================================
// |                        LOOP                          |
// ========================================================

void loop() {
  boolean updateDisplayLocal = true;
  boolean saveEEpromLocal = false;
  unsigned long secLocal;
  unsigned char sreg;

  // Interrupt free context to update shared volatile variables
  sreg = SREG;
  cli();
  updateDisplayLocal = updateDisplay;
  saveEEpromLocal = saveEEprom;
  updateDisplay = false;
  saveEEprom = false;
  secLocal = sec;
  sei();
  SREG = sreg;

  // Buzzer management
  buzzer((secLocal == 0 && timerRunning) || buzzerManCmd);
  buzzerManCmd = false;

  if (secLocal == 0) {
    START_STOP_TIMER(sreg, 0);
    timerRunning = false;
  }

  // Fix for wrong value displayed when idle: refresh display when idle, not more than
  // every UPDATE_DISPLAY_INT ms
  if (!updateDisplayLocal and !timerRunning and millis() - upDisplayTime > UPDATE_DISPLAY_INT) {
    disManager.updateAll();
    upDisplayTime = millis();
  }

  // Update display on request and reset request bit
  if (updateDisplayLocal) {

    time.min = secLocal / 60;
    time.sec = secLocal % 60;

    // Updates all the register display in reverse order for every group
    disManager.updateAll();
    updateDisplayLocal = false;
  }

  // Save score and time in EEPROM
  if (saveEEpromLocal) {
    endOfLifeEE = !writeEEPROM();
    saveEEpromLocal = false;
  }

  // Prototype input always enabled
#ifdef PROTOTYPE
  inputEnable = true;
#endif


  // Analog input not connected
  if (!inputEnable) {
    int homeAnalogValue = analogRead(HOME_ANALOG_INPUT);
    // Delay before the next analogRead, for the analog-to-digital converter
    // to settle after the last reading
    delay(10);
    int awayAnalogValue = analogRead(AWAY_ANALOG_INPUT);
    delay(10);
    int timerAnalogValue = analogRead(TIMER_ANALOG_INPUT);
    delay(4);

    // This check works only if the analog input are kept low with a pull down
    // resistor!
    if (homeAnalogValue <= MIN_VALID_ANALOG_VALUE
        && awayAnalogValue <= MIN_VALID_ANALOG_VALUE
        && timerAnalogValue <= MIN_VALID_ANALOG_VALUE) {
      inputEnable = true;
    }
  } else {
    // Analog input connected, check for buttons pressed
    int homeAnalogValue = analogRead(HOME_ANALOG_INPUT);
    int awayAnalogValue = analogRead(AWAY_ANALOG_INPUT);
    int timerAnalogValue = analogRead(TIMER_ANALOG_INPUT);

    homeButtons.checkValue(homeAnalogValue);
    awayButtons.checkValue(awayAnalogValue);
    timerButtons.checkValue(timerAnalogValue);

#ifdef DEBUG
  Serial.print("ANALOG V = ");
  Serial.print(homeAnalogValue);
  Serial.print(" ");
  Serial.print(awayAnalogValue);
  Serial.print(" ");
  Serial.println(timerAnalogValue);
#endif

    // The analog inputs have been disconnected: "random" value will be on
    // each analog port (usually much greater than 0)
    if (homeAnalogValue > MIN_VALID_ANALOG_VALUE
        && awayAnalogValue > MIN_VALID_ANALOG_VALUE
        && timerAnalogValue > MIN_VALID_ANALOG_VALUE) {
      inputEnable = false;
    }
  }
} // End of loop
