/*
* Enlighted - Otter  -  Stm32f334 based mobile worklight.
* Copyright (C) 2018 Jan Henrik Hemsing
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
  int16_t acquisitionValue[3];                 // register that holds the acquired button values
  int16_t offsetValue[3]; // Todo - make some kind of auto calibration = {2075, 2131, 2450}
  uint8_t CBSwitch;       // color or brightness switch = 0
  uint8_t state;
  uint16_t isTouchedTime;   
  uint8_t isReleased;        // power button value = 1

};

struct touch_slider_t {
  int16_t acquisitionValue[3];                 // register that holds the acquired slider values
  int16_t offsetValue[3]; // offset values which needs to be subtracted from the acquired values = {1153, 1978, 1962}
  uint16_t pos;     // current slider position = 0
  uint8_t isTouched;// 1 if slider is touched = 0
};

struct touch_t {
  uint8_t IdxBank;
  struct touch_slider_t slider;
  struct touch_button_t button;
};

// REGULATOR
struct reg_val_t {
  float target;  // Coldwhite target current in mA
  float iavg;
  float error;
  float duty;
  float iout;
};

struct reg_t {
  float Magiekonstante;
  struct reg_val_t WW;
  struct reg_val_t CW;
};

// STATUS

struct status_t {
  uint16_t ledTemp;
  uint16_t vIn;
  uint16_t iIn;
  uint16_t vBat;
  uint16_t iBat;
  uint16_t batTemp;
};

// UI
struct UI_t {
  int16_t distance;      // delta slider position
  int16_t distanceOld;
  float brightness;      // calculated brightness delta
  float brightnessAvg;      // calculated brightness delta
  float color;  // a value from 0.0f to 1.0f defining the current color porportions
  float colorAvg;  // a value from 0.0f to 1.0f defining the current color porportions
  uint8_t debounce;
};
