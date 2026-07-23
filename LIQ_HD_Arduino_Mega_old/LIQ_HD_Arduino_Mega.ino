/********************************************************
  LIQ HD
  Written by Nicholas Petersen
  https://github.com/nickpetersen93/LIQ_HD

  January, 2023

  This work includes libraries with the following licensing:
  Adafruit GFX Library - Written by Limor Fried/Ladyada for Adafruit Industries. BSD License
  Adafruit ILI9341 Library - Written by Limor Fried/Ladyada for Adafruit Industries. MIT license. BSD License.
  Adafruit FT6206 Library - Written by Limor Fried/Ladyada for Adafruit Industries. MIT license.
  Adafruit MPR121 Library - Written by Limor Fried/Ladyada for Adafruit Industries. MIT license. BSD License.
  RTCLib Library - Written by JeeLabs. MIT license.

  This work is released under This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License:
  https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode

  Copyright (c) 2023 Nicholas Petersen. Creative Commons BY-NC-SA License. Non-Commercial Use Only.
  Adapter's License must be a Creative Commons BY-NC-SA or Compatible License.
  All text above must be included in any redistribution. See LICENSE.md for detailed information.

  UNLESS OTHERWISE SEPARATELY UNDERTAKEN BY THE LICENSOR, TO THE EXTENT POSSIBLE, THE LICENSOR OFFERS
  THE LICENSED MATERIAL AS-IS AND AS-AVAILABLE, AND MAKES NO REPRESENTATIONS OR WARRANTIES OF ANY KIND
  CONCERNING THE LICENSED MATERIAL, WHETHER EXPRESS, IMPLIED, STATUTORY, OR OTHER. THIS INCLUDES,
  WITHOUT LIMITATION, WARRANTIES OF TITLE, MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
  NON-INFRINGEMENT, ABSENCE OF LATENT OR OTHER DEFECTS, ACCURACY, OR THE PRESENCE OR ABSENCE OF ERRORS,
  WHETHER OR NOT KNOWN OR DISCOVERABLE. WHERE DISCLAIMERS OF WARRANTIES ARE NOT ALLOWED IN FULL OR IN
  PART, THIS DISCLAIMER MAY NOT APPLY TO YOU.

  TO THE EXTENT POSSIBLE, IN NO EVENT WILL THE LICENSOR BE LIABLE TO YOU ON ANY LEGAL THEORY (INCLUDING,
  WITHOUT LIMITATION, NEGLIGENCE) OR OTHERWISE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL,
  CONSEQUENTIAL, PUNITIVE, EXEMPLARY, OR OTHER LOSSES, COSTS, EXPENSES, OR DAMAGES ARISING OUT OF THIS
  PUBLIC LICENSE OR USE OF THE LICENSED MATERIAL, EVEN IF THE LICENSOR HAS BEEN ADVISED OF THE
  POSSIBILITY OF SUCH LOSSES, COSTS, EXPENSES, OR DAMAGES. WHERE A LIMITATION OF LIABILITY IS NOT
  ALLOWED IN FULL OR IN PART, THIS LIMITATION MAY NOT APPLY TO YOU.

  THE DISCLAIMER OF WARRANTIES AND LIMITATION OF LIABILITY PROVIDED ABOVE SHALL BE INTERPRETED IN A
  MANNER THAT, TO THE EXTENT POSSIBLE, MOST CLOSELY APPROXIMATES AN ABSOLUTE DISCLAIMER AND WAIVER OF
  ALL LIABILITY.


  ------------------------------------------------------------------------------------------
  MODIFIED: one MPR121 / one cage / two bottles, rewritten lick detector.
  See README_LICK_DETECTION_FIX.md for what changed and why.
  ------------------------------------------------------------------------------------------
********************************************************/

#include <Adafruit_GFX.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include "RTClib.h"
#include "Adafruit_MPR121.h"
#include <Adafruit_ILI9341.h>
#include <Adafruit_FT6206.h>
#include <math.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans18pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>


//#define DEBUG //uncomment to turn on debug mode, prints info to Serial Monitor

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTDEC(x) Serial.print(x, DEC)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINTLN(x)
#endif

// The FT6206 uses hardware I2C on the shield
Adafruit_FT6206 ts = Adafruit_FT6206();

// The display also uses hardware SPI, plus #9 & #10
#define TFT_CS 10
#define TFT_DC 9
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

RTC_PCF8523 rtc;           // define the Real Time Clock object
const int chipSelect = 7;  // for the data logging shield, changed from digital pin 10
#define redLEDpin 13
DateTime now;

//capacitive touch device
#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif

Adafruit_MPR121 cap = Adafruit_MPR121();
Adafruit_MPR121 cap2 = Adafruit_MPR121();
Adafruit_MPR121 cap3 = Adafruit_MPR121();


// ==========================================================================================
//  SECTION 1 - WIRING
// ==========================================================================================
#define ONE_PAIR_MODE true
#define ACTIVE_MPR121_ADDRESS 0x5B   // your I2C scanner found the board here
#define ACTIVE_SIPPER_COUNT 2
#define ACTIVE_CAGE_COUNT 1
#define LEFT_ELECTRODE 0             // MPR121 E0 -> left bottle  -> column 1
#define RIGHT_ELECTRODE 1            // MPR121 E1 -> right bottle -> column 2


// ==========================================================================================
//  SECTION 2 - I2C BUS SPEED  *** IMPORTANT ***
//
//  The stock LIQ HD code calls Wire.setClock(3400000). On a 16 MHz AVR that value
//  overflows the TWBR calculation and the bus actually ends up running at about
//  31 kHz - THREE TIMES SLOWER than standard 100 kHz mode. That alone limits the
//  lick sampling rate to roughly 50 Hz, which is too slow to reliably catch a
//  30-50 ms mouse lick.
//
//  400000 is the correct fast-mode value and is supported by the MPR121, the
//  PCF8523 RTC and the FT6206 touch controller. Drop to 100000 only if you start
//  seeing I2C timeouts on the screen counter.
// ==========================================================================================
#define I2C_CLOCK_HZ 400000UL


// ==========================================================================================
//  SECTION 3 - MPR121 ANALOG FRONT END
//
//  The MPR121 charges the electrode with a constant current (CDC) for a fixed time
//  (CDT) and measures the resulting voltage: V = I * t / C.
//  A lick adds a small capacitance dC, so the reading drops by roughly
//
//        delta_counts  =  filtered_value * (dC / C)
//
//  Two consequences that drive this whole rewrite:
//   1. The bigger your resting "filtered value", the bigger every lick is in counts.
//      So set CDC/CDT to put the resting value high, and your signal scales up for free.
//   2. The bigger your baseline electrode capacitance C (long wire, big copper band,
//      full tube of liquid), the SMALLER every lick is. Shrinking C is a hardware fix
//      and it is the single most effective thing you can do.
//
//  Leave auto gain on. It sweeps CDC/CDT at startup and picks the combination that
//  puts the resting reading closest to mpr_target_filtered.
// ==========================================================================================
bool mpr_auto_gain = true;        // sweep CDC/CDT at startup and pick the best
int  mpr_target_filtered = 700;   // aim resting filteredData here (0-1023). 600-800 is good.

// Used only when mpr_auto_gain = false. Values chosen by the tuner sketch.
int  mpr_manual_cdc = 63;         // charge current, 1-63 uA
int  mpr_manual_cdt_code = 4;     // 1=0.5us 2=1us 3=2us 4=4us 5=8us 6=16us 7=32us

// First filter iterations: 0=6 samples, 1=10, 2=18, 3=34. Higher = quieter, slightly slower.
int  mpr_ffi_code = 2;
// Second filter iterations: 0=4 samples, 1=6, 2=10, 3=18.
int  mpr_sfi_code = 0;
// Electrode sample interval: 0=1ms, 1=2ms, 2=4ms, 3=8ms ... keep at 0.
int  mpr_esi_code = 0;


// ==========================================================================================
//  SECTION 4 - LICK DETECTION
//
//  Detection now runs on filteredData with a software baseline, NOT on
//  (baselineData - filteredData). The Adafruit library returns baselineData as an
//  8-bit register shifted left by 2, so that difference can only move in steps of 4.
//  Any threshold of 1, 2 or 3 counts was therefore reacting to baseline quantisation
//  steps rather than to your mouse. filteredData is the full 10-bit value.
//
//  Leave auto_threshold on for your first run. It measures the actual idle noise of
//  your rig and sets the threshold from it, which is the only sane way to pick a number.
// ==========================================================================================
bool auto_threshold = false;             // set thresholds from measured noise at calibration
float auto_threshold_sd_multiple = 6.0; // threshold = this many standard deviations of idle noise
int  min_touch_threshold = 6;           // never go below this, even on a very quiet rig
int  max_touch_threshold = 90;          // if auto lands above this your hardware needs work
int  release_percent = 55;              // release threshold as a % of touch threshold (hysteresis)

// These two are also editable on the touchscreen Settings page. When auto_threshold is
// true they are OVERWRITTEN at calibration. Set auto_threshold = false to keep your own.
int touch_threshold = 5;    // delta in filteredData counts to call a touch
int release_threshold = 2;   // must be lower than touch_threshold

// --- time-domain gating: this is what kills electrical noise without killing licks ---
// A mouse licks at 6-9 Hz. Each tongue contact lasts roughly 30-70 ms and the gap
// between contacts is roughly 60-110 ms. Electrical noise does not look like that.
int min_lick_ms = 12;        // contact must be held this long before it counts as a lick
int refractory_ms = 25;      // ignore a new onset for this long after a release
int max_lick_ms = 1500;      // longer than this = stuck channel; force release and rebaseline

// --- software baseline tracker ---
int lick_sample_interval_ms = 4;   // MPR121 updates filteredData every ~4 ms by default
int baseline_shift_slow = 11;      // drift tracking, ~8 s time constant
int baseline_shift_fast = 5;       // fast re-acquire when signal jumps ABOVE baseline
int baseline_freeze_ms = 2000;     // stop tracking for this long after any lick activity
int fast_recover_delta = 25;       // counts above baseline that trigger fast re-acquire

// --- calibration ---
int cal_settle_ms = 1200;          // let the analog filters settle before measuring
int cal_samples = 250;             // idle samples used to measure noise


// ==========================================================================================
//  SECTION 5 - DISPLAY AND DEBUG
// ==========================================================================================
bool force_full_brightness = true;   // keep screen bright during bench testing
int screen_brightness_day = 255;
int screen_brightness_night = 1;
bool display_live_current_bin = true;  // show current-bin licks immediately, not only after the CSV write

// Serial Monitor / Serial Plotter at 115200.
//   0 = off (normal recording)
//   1 = one line per detected lick (channel, duration, peak delta)
//   2 = continuous stream of delta values, good for Serial Plotter
int serial_sensor_debug = 0;
int serial_debug_interval_ms = 20;


// ==========================================================================================
//  SECTION 6 - LIVE  touch=YES/no  READOUT
//
//  Prints the touch state of both bottles to the Serial Monitor at 115200 while
//  recording, so you can watch the detector make its decision in real time.
//
//  ---- TO REMOVE THIS FEATURE COMPLETELY, COMMENT OUT THE NEXT LINE ----
//  With it commented out none of this code is compiled at all: no flash used,
//  no RAM used, and nothing added to the recording loop.
// ==========================================================================================
#define SHOW_TOUCH_STATE

#ifdef SHOW_TOUCH_STATE
// Runtime on/off. Lets you silence the readout without recompiling the logic away.
bool touch_state_enabled = true;

// true  = print one line each time a bottle flips between touched and not touched.
//         Quiet and easy to read. This is what you usually want.
// false = print a line every touch_state_interval_ms regardless. Use this when you
//         want to confirm the loop is alive even though nothing is being touched.
bool touch_state_on_change_only = true;

int touch_state_interval_ms = 250;   // only used when touch_state_on_change_only = false

// Also show the measured delta and the current threshold next to YES/no.
// This is what tells you WHY it decided the way it did.
bool touch_state_show_numbers = true;
#endif


// ==========================================================================================
//  Internal detector state. Do not edit.
// ==========================================================================================
#define LK_NCH 2
#define LK_IDLE 0
#define LK_CANDIDATE 1
#define LK_TOUCH 2

int32_t  lk_baseline_q[LK_NCH];
int16_t  lk_filtered[LK_NCH];
int16_t  lk_delta[LK_NCH];
int16_t  lk_peak_delta[LK_NCH];
uint8_t  lk_state[LK_NCH];
unsigned long lk_cand_start[LK_NCH];
unsigned long lk_touch_start[LK_NCH];
unsigned long lk_last_release[LK_NCH];
unsigned long lk_last_event[LK_NCH];
unsigned long lk_next_sample_ms = 0;
unsigned long lk_active_ms = 0;
unsigned long lk_rejected[LK_NCH];   // sub-threshold-duration blips thrown away
unsigned long lk_stuck[LK_NCH];      // forced releases
float    lk_noise_sd[LK_NCH];
int      lk_idle_level[LK_NCH];
bool     pair_raw_calibrated = false;
unsigned long last_serial_sensor_debug_ms = 0;
// ==========================================================================================


// Keeps track of the last pins touched so we know when buttons are 'released'
uint16_t lasttouched1 = 0;
uint16_t currtouched1 = 0;
uint16_t lasttouched2 = 0;
uint16_t currtouched2 = 0;
uint16_t lasttouched3 = 0;
uint16_t currtouched3 = 0;

File logfile;  // the logging file
char filename[] = "00000000.CSV";

TS_Point p;
extern uint8_t welcome_logo[];
extern uint8_t main_logo[];
extern uint8_t settings_icon[];
extern uint8_t settings_icon2[];
unsigned long startmillis;
String E_side = "RIGHT";
String display_page = "main";
bool show_time = true;
bool eject_button;
int LOG_COUNTER;
int SYNC_COUNTER;
unsigned long currentMillis;
unsigned long previousMillis;
int syncTime;
int min_now;
int prev_min;
int curr_min;
int hour_now;
bool refresh_page = true;
int timeouts = 0;
bool ok;
int default_lights_on;
int default_lights_off;
int default_touch_threshold;
int default_release_threshold;
int default_LOG_INTERVAL;
int dafault_SYNC_INTERVAL;
bool default_auto_cal;
int default_auto_cal_time;
int default_auto_cal_sec_since_last_lick;
bool auto_cal_flag = true;
unsigned long cal_timer;
String logging[8];
bool log_LN = true;
bool log_LD = true;
bool log_BN = true;
bool log_BD = true;
bool log_BLN = true;
bool log_BLD = true;
bool recording = false;
String licked_bottle;

// cached RTC minute, so we stop hammering the I2C bus three times per loop
unsigned long rtc_cache_ms = 0;
int rtc_cache_interval_ms = 250;

//enough for 18 cages, 2 lickometers per cage
unsigned long LickDuration[36];
unsigned long LickNumber[36];
unsigned long BoutNumber[36];
unsigned long BoutDuration[36];
unsigned long BoutLickNumber[36];
unsigned long BoutLickDuration[36];
unsigned long BoutLickDuration_bytime[36];
float LickFrequency[36];
float ILI[36];
unsigned long Elapsedtime[36];
unsigned long total_LN[36];
int lick_bout_countdown[36];
unsigned long bout_timer[36];
unsigned long bout_start_timer[36];
bool in_bout[36];
bool licking[36];
unsigned long time_now[36];
unsigned long last_lick_time[36];

// how many minutes between grabbing data and logging it.
int LOG_INTERVAL = 1;  // IN MINUTES! between entries (reduce to take more/faster data)
// how many milliseconds before writing the logged data permanently to disk
int SYNC_INTERVAL = 10;                   // IN MINUTES! between calls to flush()
unsigned long emergency_counter = 60050;  //1 minute +50ms

// Kept for compatibility with the stock settings page. debounce is a hardware
// MPR121 setting that only affects cap.touched(), which we no longer use for licks.
int debounce = 0;
int sensor_charge_current = 16;
bool auto_cal = true;       //daily auto recalibration
int auto_cal_time = 7;      //hour of day when auto cal happens (light cycle, animals less active)
int auto_cal_sec_since_last_lick = 60;  //seconds of inactivity required before auto cal
int bout_cutoff = 3000;     //bout cut off time in milliseconds
bool log_by_bout = false;   //ONLY ONE SHOULD BE TRUE
bool log_by_time = true;    //ONLY ONE SHOULD BE TRUE

int lights_on = 6;    //time of day (24hr) housing lights turn ON
int lights_off = 18;  //time of day (24hr) housing lights turn OFF

//==========================================================================================

void setup() {
  // Start the serial port if anything at all wants to print.
  bool want_serial = (serial_sensor_debug > 0);
#ifdef DEBUG
  want_serial = true;
#endif
#ifdef SHOW_TOUCH_STATE
  if (touch_state_enabled) want_serial = true;
#endif
  if (want_serial) Serial.begin(115200);

  pinMode(5, OUTPUT);  // set screen brightness pin as output
  analogWrite(5, screen_brightness_day);

  pinMode(redLEDpin, OUTPUT);
  digitalWrite(redLEDpin, LOW);

  set_defaults();  //save coded settings as the default settings

  start_display();  //initialize display

  start_rtc();  //iniitialize rtc

  WelcomeScreen();  //display welcome screen at startup

  set_brightness();  //set screen brightness depending on light cycle
}

//==========================================================================================

void loop() {

  main_menu_page();

  recording_page();

  pause_page();

  SD_error_page();

  settings_page();

  error_page();
}
