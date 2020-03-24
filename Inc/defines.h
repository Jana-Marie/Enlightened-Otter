/*
 * Enlightened-Otter  -  Stm32f334 based mobile worklight.
 * Copyright (C) 2019 Jan Henrik Hemsing
 *
 * This program is free software: you can redistribute it and/or modify it
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
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DO_EXPAND(VAL)  VAL ## 1
#define EXPAND(VAL)     DO_EXPAND(VAL)

#define SCOPE_CHANNELS 	0 	// sets the number of (uart) scope channels to be set/transmitted, non defining SCOPE_CHANNELS will remove the function completly

#define MAX_CURRENT 499.0f 	// sets the maximum current

#define HRTIM_FREQUENCY_KHZ 500.0f 	// sets the frequency of the PWM output channels maximum frequency (8 bit PWM): 18Mhz (18000.0) SHOULD BE DIVIDABLE BY 2
 									// 250.2f,300.0f,350.0f,400.0f,450.0f,500.0f,550.0f,600.0f,650.1f,700.1f,750.0f,800.0f
#define REG_CNT 		10	 // sets the number of HRTIM passes to the next controller pass
#define KI 					0.4f // sets the KI constant for the current regulator - do not change unless you know what you're doing

#define MIN_DUTY 	0.007f 	// sets the minimum duty cycle that the regulation can reach, can be left at 0.002
#define MAX_DUTY 	0.83f 	// sets the maximum duty cycle that the regulation can reach, should not exceed a certain but by now uncertain value

#define CURRENT_AVERAGING_FILTER 	0.90f	// koeffizient of current averaging filter 0 = no averaging 1 = infinite averaging
#define COLOR_FADING_FILTER 		  0.90f	// koeffizient of color cross fading filter
#define BRIGHTNESS_FADING_FILTER 	0.99f	// koeffizient of brightness fading filter
#define TOUCH_THRESHOLD_FILTER		0.55f

#define STANDBY_TIME      4     // turn off EO after 4 min idle
#define TURNOFF_TIME 		  60		// sets the time to count to before turning off
#define BUTTON_THRESHOLD  -1300	// sets the threshold at which a button press has to be triggered
#define IS_TOUCHED_ABS    -700  // sets the threshold for a slider detection event
#define IS_RELEASED_ABS   -500 // sets the threshold for slider release

#define CURRENT_CUTOFF		0.6f 	// sets the threshold at which the boost will be turned off

#define SLIDER_BEHAVIOR		AB 	// 0 = AB = Absolute, 1 = REL = relative

#define SHUNT         0.22f
#define SHUNT_SERIE   1000
#define SHUNT_PULLUP  220000
#define AREF          2.9f
#define ARES          4096.0f
#define SHUNT_GAIN    19.3333f
#define BATT_PULLUP   220000.0f
#define BATT_PULLDOWN 47000.0f

#define FAULT_CURRENT		(int)(3900) 	// Overcurrent set to ~590mA
#define FAULT_VOLTAGE		(int)(2250) 	// Overvoltaeg set to 17V
/*
#if (EXPAND(SLIDER_BEHAVIOR) == 1)
#define TOUCH_SCALE 200.0f   	// sets the touch slider scale - I have No idea anymore how this works
#else
#define TOUCH_SCALE 62.5f   	// sets the touch slider scale - I have No idea anymore how this works
#endif
*/
#define TOUCH_SCALE     62.5f
#define UI_DEBOUNCE_VAL 4

// ############################################################# //
// Automatic calculated Values, please use the variables above

#define HRTIM_PERIOD      (int)(1.0/(HRTIM_FREQUENCY_KHZ*1000.0f)/0.000000000217f) // calculates the timer period value, therefore sets the frequency
#define STANDBY_TIME_CALC (uint32_t)(STANDBY_TIME*60*1000*2)
#define LED_BUFFER_SIZE   ((6*24) + 42)

// ############################################################# //
// I2C registers

#define RT_ADDRESS 			(0x53 << 1)	// std i2c address for the rt9466

#define CHG_CTRL1 				0x01	//
#define ENABLE_OTG_MASK 		0x11	// enables OTG, should be on by default
#define DISABLE_OTG_MASK 		0x10	// disables OTG, should be on by default
#define ENABLE_STAT_LED_MASK 	0x10	// Turn on status LED (default)
#define DISABLE_STAT_LED_MASK 	0x00	// turn off status LED

#define CHG_CTRL2 		0x02	//
#define TURNOFF_MASK 	0x83	// shuts down power immediately, can only be resetted by button press
#define IINLIM_MASK 	0x0B	// sets the input current limit to software register see CHG_CTRL3

#define CHG_CTRL3 		0x03	//
#define SET_ILIM_3A 	0xEB	// sets the input current limit to 3A

#define CHG_CTRL13		0x0D	//
#define ENABLE_UUG		0x52 	// Enables UUG_Fet, battery voltage is present on VMID
#define DISABLE_UUG		0x50 	// Disables UUG_Fet, no Battery voltage is present on VMID nor the OTG voltage is present on VBUS

#define CHG_CTRL16 		0x10	//
#define DISABLE_JEITA 	0x00	// Disable JEITA charger profile, idk maybe we dont want this
#define ENABLE_JEITA 	0x10	// Enable it again

#define CHG_IRQ3 		0x55
#define ADC_READY_MASK	0x01

#define CHG_STAT 		0x42	// Contains all the status bits
#define FASTCHARGE_MASK 0x20
#define CHG_MASK		0x40
#define CHG_DONE_MASK	0x80
#define ADC_DONE_MASK	0x01
// [7:6] 00 ready; 01 charge in progress; 10 charge done; 11 fault
// 5 	 0 precharge; 1 fastcharge
// 4	 0 no trickle charge; 1 trickle charge
// 3 	 0 not in boos mode; 1 in boost mode
// 2 	 0 boost VBUS OVP does not occur ; 1 boost VBUS OVP occurs
// 1  	 reserved
// 0 	 0 ADC idle; 1 ADC under conversion

#define ADC_DATA_H 	0x44	// ADC High byte value
#define ADC_DATA_L 	0x45	// ADC Low byte value

#define CHG_ADC   	0x11 	//
#define ADC_VBUS5 	0x11    //lsb 25mv +-2lsb
#define ADC_VBUS2 	0x21    //lsb 10mv +-2lsb
#define ADC_VSYS  	0x31    //lsb 5mv +-2lsb
#define ADC_VBAT  	0x41    //lsb 5mv +-2lsb
#define ADC_NTC   	0x61    //lsb 0.25% +-2lsb
#define ADC_IBUS  	0x81    //lsb 50mA +-2lsb
#define ADC_IBAT  	0x91    //lsb 50mA +-2lsb
#define ADC_REGN  	0xB1    //lsb 5mv +-2lsb
#define ADC_TEMP_JC 0xC1 	//lsb 2degreeC +-2lsb

// ############################################################# //
// Misc

#define UART_DMA_CHANNEL DMA1_Channel4

#define CHAN_WW 0
#define CHAN_CW 1

#define AB 0
#define REL 1

#define LED_CMPH 50
#define LED_CMPL 26

// ############################################################# //
// usefull functions

#define ADC2VBAT(a) (((a / ARES) * AREF)*((BATT_PULLUP + BATT_PULLDOWN) / BATT_PULLDOWN))
#define AMP(a, gain) (((a) * AREF / ARES / (gain) - AREF / (SHUNT_PULLUP + SHUNT_SERIE) * SHUNT_SERIE) / (SHUNT * SHUNT_PULLUP) * (SHUNT_PULLUP + SHUNT_SERIE))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define SCALE(value, high, max) MIN(MAX(((max) - (value)) / ((max) - (high)), 0.0), 1.0)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define FILT(a, b, c) ((a) * (c) + (b) * ((1.0f) - (c)))
#define ABS(x) ((x)<0 ? -(x) : (x))
