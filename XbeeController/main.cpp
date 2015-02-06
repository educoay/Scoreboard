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
 * Xbee controller main file.
 * ATTiny85V source code.
 * Scoreboard Project - Wireless branch.
 * version 1.0.0
 */

#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

// Analog input rows port IDs
#define HOME_ANALOG_INPUT       A1
#define AWAY_ANALOG_INPUT       A2
#define TIMER_ANALOG_INPUT      A3

// Digital outputs
#define WAKE_OUTPUT             1

// Constants
#define MIN_ANA_TH              25      // Minimum valid analog value [0 - 1023]
#define WD_CMP_MATCH            5       // Watchdog interrupt count before putting Xbee to sleep
                                        // wd count * wd interval >= xbee sample before TX * xbee sample rate IR
                                        // 5 * 16ms >= 2 * 40ms

// Routines to set and clear bits (used in the sleep code)
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// ####################### Prototypes #######################
// Setup watchdog wakeup interrupt
void setup_watchdog(int ii);

// Set system into the sleep state,
// system wakes up when watchdog is timed out
void system_sleep();

// ####################### Variables ########################

volatile byte wdCount = 0;

void setup_watchdog(int ii) {
  // 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
  // 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
  byte bb;

  if (ii > 9)
    ii = 9;
  bb = ii & 7;

  if (ii > 7)
    bb |= (1 << 5);
  bb |= (1 << WDCE);

  MCUSR &= ~(1 << WDRF);

  // Start timed sequence
  WDTCR |= (1 << WDCE) | (1 << WDE);

  // Set new watchdog timeout value
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
}

void system_sleep() {
  cbi(ADCSRA, ADEN);                        // Switch Analog to Digitalconverter OFF
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);      // Sleep mode is set here
  sleep_enable();
  sleep_mode();                             // System actually sleeps here
  sleep_disable();                          // System continues execution here when watchdog timed out
  sbi(ADCSRA, ADEN);                        // Switch Analog to Digitalconverter ON
}

// Watchdog Interrupt Service Routine
ISR(WDT_vect) {
  wdCount++;
}

// ========================================================
// |                        SETUP                         |
// ========================================================

void setup() {
  pinMode(WAKE_OUTPUT, OUTPUT);
  digitalWrite(WAKE_OUTPUT, HIGH);

  // Setup watchdog timeout
  setup_watchdog(0);

  // Set the internal registers to reduce power consumes
  PRR &= ~(1<<PRTIM1);                      // Shut down the timer1
  ACSR = (1<<ACD);                          // Shut down the analog comparator
  MCUCR |= (1<<BODS);                       // BOD disabled

} // End of setup

// ========================================================
// |                        LOOP                          |
// ========================================================

void loop() {

  system_sleep();                           // Send the unit to sleep

  boolean pressedHome = (analogRead(HOME_ANALOG_INPUT) > MIN_ANA_TH);
  boolean pressedAway = (analogRead(AWAY_ANALOG_INPUT) > MIN_ANA_TH);
  boolean pressedTimer = (analogRead(TIMER_ANALOG_INPUT) > MIN_ANA_TH);

  if (pressedHome || pressedAway || pressedTimer) {
    digitalWrite(WAKE_OUTPUT, LOW);
    wdCount = 0;
  } else if (wdCount >= WD_CMP_MATCH) {
    digitalWrite(WAKE_OUTPUT, HIGH);
    wdCount = 0;
  }

} // End of loop
