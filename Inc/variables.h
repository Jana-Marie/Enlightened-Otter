/*
* Enlightened - Otter  -  Stm32f334 based mobile worklight.
* Copyright (C) 2019 Jan Henrik Hemsing
*
* This program is free software: you can redistribute it and / or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 3 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of  MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along with
* this program.  If not, see < http : //www.gnu.org/licenses/>.
*/

// TOUCH
struct touch_button_t {
  int16_t acquisitionValue[3];    // register that holds the acquired button values
  int16_t offsetValue[3];         // register that stores the correction values for all buttons
  uint8_t CBSwitch;               // Stores if color or brightness button was pressed
  uint8_t state;                  // stores the power state
  uint32_t isTouchedTime;         // stores the time for how long the power button was pressed
  uint32_t isTouchedTime2;         // stores the time for how long the other buttons were pressed
  uint32_t isTouchedTime3;         // stores the time for how long the other buttons were pressed
  uint8_t doubleTap[2];
  uint8_t isReleased;             // stores if power button was released
};

struct touch_slider_t {
  int16_t acquisitionValue[3];    // register that holds the acquired slider values
  int16_t offsetValue[3];         // register that stores the correction values for the slider
  int16_t pos;                    // current slider position
  int16_t posAvg;                 // current average slider position
  int16_t posLast;                // last slider position
  int16_t isTouchedValDelta;      // current slider position
  int16_t isTouchedValAvg;        // current slider position
  int16_t isTouchedVal;           // current slider position
  uint8_t isTouched;              // 1 if slider is touched
};

struct touch_t {                  // stores the touch states
  uint8_t IdxBank;                // iterates throught the banks/IOs
  struct touch_slider_t slider;
  struct touch_button_t button;
};

// REGULATOR
struct reg_val_t {
  float target;         // target current in mA
  //float targetNoGamma;  // target current without gamma correcture
  float ioff;
  float iout;           // LED current current in mA
  float iavg;           // LED current (averaged) in mA
  float error;          // error current in mA
  float duty;           // output duty cycle
};

struct reg_t {          // contains the values for both regulators
  float Magiekonstante; // stores the KI val
  struct reg_val_t WW;
  struct reg_val_t CW;
};

// STATUS
struct status_t {   // status storage struct
  float ledTemp; // contains led temperature
  float vIn;     // contains input voltage
  float iIn;     // contains input current
  float vBat;    // contains battery voltage
  float vBatRt;  // contains battery voltage measured by ritchtek
  float iBat;    // contains battery current
  float pIn;     // contains the power drawn from the USB-port
  float pBat;    // contains the power drawn from the battery
  float pSum;    // contains the sum of the power drawn
  float batTemp; // contains battery temperature
  int8_t  state;    // defines state of state maschine
  uint8_t errCnt;
};

// UI
struct UI_t {           // storage for ui task
  int16_t distance;     // contains current slider position
  int16_t distanceOld;  // contains the old slider position (after 1 cycle)
  float brightness;     // calculated target brightness
  float brightnessLast[4];  // calculated target brightness averaged
  float brightnessAvg;
  float color;          // a value from 0.0f to 1.0f defining the current color porportions
  float colorLast[4];       // a value from 0.0f to 1.0f defining the current color porportions averaged
  float colorAvg;
  uint8_t debounce;     // debounce counter
};
