/*
 * Enlighted-Otter  -  Stm32f334 based mobile worklight.
 * Copyright (C) 2018 Jan Henrik Hemsing
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

#define SCOPE_CHANNELS 	7 	// sets the number of (uart) scope channels to be set/transmitted, non defining SCOPE_CHANNELS will remove the function completly

#define MAX_CURRENT 400.0f 	// sets the maximum current - warning, because of gamma correction 

// values to be tested to determine the most efficient boost converter frequency
// 250.2f,300.0f,350.0f,400.0f,450.0f,500.0f,550.0f,600.0f,650.1f,700.1f,750.0f,800.0f

#define HRTIM_FREQUENCY_KHZ 750.0f 	// sets the frequency of the PWM output channels maximum frequency (8 bit PWM): 18Mhz (18000.0) SHOULD BE DIVIDABLE BY 2
#define REG_CNT 			254	 	// sets the number of HRTIM passes to the next controller pass
#define KI 					0.001f 	// sets the KI constant for the current regulator - do not change unless you know what you're doing

#define MIN_DUTY 	0.002f 	// sets the minimum duty cycle that the regulation can reach, can be left at 0.002
#define MAX_DUTY 	0.83f 	// sets the maximum duty cycle that the regulation can reach, should not exceed a certain but by now uncertain value

#define OVERVOLTAGE 18.0f 	// V  -  Voltage set for Overvoltage protection, Vtargetmax is ~16.5V 
#define OVERCURRENT 0.8f	// A  -  set current for overcurrent protection (LEDs are speced @100mA but can work with ~150mA)

#define CURRENT_AVERAGING_FILTER 	0.995f	// koeffizient of current averaging filter 0 = no averaging 1 = infinite averaging
#define COLOR_FADING_FILTER 		0.95f	// koeffizient of color cross fading filter
#define BRIGHTNESS_FADING_FILTER 	0.95f	// koeffizient of brightness fading filter
#define TOUCH_FILTER				0.75f

#define POWER_LED_BRIGHTNESS 64		// brightness of the power LED in off state (0-1024)

#define TURNOFF_TIME 		130		// sets the time to count to before turning off 
#define BUTTON_THRESHOLD 	-950	// sets the threshold at which a button press has to be triggered
#define SLIDER_THRESHOLD 	-350	// sets the threshold at which the slider reports a value

// ############################################################# //
// Automatic calculated Values, please use the variables above

#define TOUCH_SCALE ((MAX_CURRENT/200.0f)*22.2996515f)   	// sets the touch slider scale 22.2996515 -> 0-200 (TOUCH_SCALE * 8.96875)

#define VDDA 				3.0f 	// Vref = VDDA = Analog power supply
#define CURRENT_PRESCALER	1.0f 	// set the divisor for the current input
#define VOLTAGE_PRESCALER	7.47f	//set the divisor for the voltage input
#define FAULT_CURRENT		(int)(4096*((OVERCURRENT/CURRENT_PRESCALER)/VDDA)) 	// calculates the value the DAC for the Overcurrent protection has to be set to
#define FAULT_VOLTAGE		(int)(4096*(((OVERVOLTAGE-0.7f)/VOLTAGE_PRESCALER)/VDDA)) 	// calculates the value the DAC for the Overvoltage protection has to be set to
#define HRTIM_PERIOD 		(int)(1.0/(HRTIM_FREQUENCY_KHZ*1000.0f)/0.000000000217f) // calculates the timer period value, therefore sets the frequency

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

#define CHG_STAT 		0x42	// Contains all the status bits
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

// ############################################################# //
// usefull functions

#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define SCALE(value, high, max) MIN(MAX(((max) - (value)) / ((max) - (high)), 0.0), 1.0)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define FILT(a, b, c) ((a) * (c) + (b) * ((1.0f) - (c)))
#define ABS(x) ((x)<0 ? -(x) : (x))