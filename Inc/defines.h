#define SCOPE_CHANNELS 	6 	// sets the number of (uart) scope channels to be set/transmitted, non defining SCOPE_CHANNELS will remove the function completly

#define HRTIM_FREQUENCY_KHZ 450.0f // sets the frequency of the PWM output channels maximum frequency (8 bit PWM): 18Mhz (18000.0)  

#define MIN_DUTY 0.002f // sets the minimum duty cycle that the regulation can reach, can be left at 0.002
#define MAX_DUTY 0.83f 	// sets the maximum duty cycle that the regulation can reach, should not exceed a certain but by now uncertain value

#define OVERVOLTAGE 18.0f 	// V  -  Voltage set for Overvoltage protection, Vtargetmax is ~16.5V 
#define OVERCURRENT 0.300f	// A  -  set current for overcurrent protection (LEDs are speced @100mA but can work with ~150mA)

// Automatic calculated Values, please use the variables above

#define VDDA 				3.0f 	// Vref = VDDA = Analog power supply
#define CURRENT_PRESCALER	1.0f 	// set the divisor for the current input
#define VOLTAGE_PRESCALER	7.47f	//set the divisor for the voltage input
#define FAULT_CURRENT		4096*((OVERCURRENT/CURRENT_PRESCALER)/VDDA) 	// calculates the value the DAC for the Overcurrent protection has to be set to
#define FAULT_VOLTAGE		4096*(((OVERVOLTAGE-0.7f)/VOLTAGE_PRESCALER)/VDDA) 	// calculates the value the DAC for the Overvoltage protection has to be set to
#define HRTIM_PERIOD 		(1.0/(HRTIM_FREQUENCY_KHZ*1000.0f)/0.000000000217f) // calculates the timer period value, therefore sets the frequency

#define RT_ADDRESS 			(0x53 << 1)	// std i2c address for the rt9466

#define CHG_CTRL1 				0x01	// 
#define ENABLE_OTG_MASK 		0x11	// enables OTG, should be on by default
#define ENABLE_STAT_LED_MASK 	0x10	// Turn on status LED (default)
#define DISABLE_STAT_LED_MASK 	0x00	// turn off status LED

#define CHG_CTRL2 		0x02	//
#define TURNOFF_MASK 	0x83	// shuts down power immediately, can only be resetted by button press
#define IINLIM_MASK 	0x0B	// sets the input current limit to software register see CHG_CTRL3

#define CHG_CTRL3 	0x03	//
#define SET_ILIM_3A 0xEB	// sets the input current limit to 3A

#define CHG_CTRL16 		0x10	//
#define DISABLE_JEITA 	0x00	// Disable JEITA charger profile, idk maybe we dont want this
#define ENABLE_JEITA 	0x10	// Enable it again

#define CHG_STAT 0x42		// Contains all the status bits
// [7:6] 00 ready; 01 charge in progress; 10 charge done; 11 fault
// 5 	 0 precharge; 1 fastcharge
// 4	 0 no trickle charge; 1 trickle charge
// 3 	 0 not in boos mode; 1 in boost mode
// 2 	 0 boost VBUS OVP does not occur ; 1 boost VBUS OVP occurs
// 1  	 reserved
// 0 	 0 ADC idle; 1 ADC under conversion

#define ADC_DATA_H 0x44		// ADC High byte value
#define ADC_DATA_L 0x45		// ADC Low byte value

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

#define TOUCH_SCALE 16   	// sets the touch slider scale 16 -> 0-100

#define UART_DMA_CHANNEL DMA1_Channel4

// usefull functions
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define SCALE(value, high, max) MIN(MAX(((max) - (value)) / ((max) - (high)), 0.0), 1.0)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))