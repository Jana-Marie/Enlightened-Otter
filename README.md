# Enlighted-Otter

Enlighted-Otter is an Open-Source (soon also OSHW) worklight for hacker/maker events like the chaos communication congress. It is based upon a STM32F334 with its high resolution timer as dual boost 
converter. The main goal is to provide cableless, high CRI, high brightness, flicker-free illumination with a variable color temperature. This is achieved by using LEDs with high CRI (>93) and the boost converter operating at a frequency of up to 850khz. The PCB also features USB-C (power profile 1) to deliver enought power to charge the 18650 type batteries as well as keep up operation of the boost converter.

Enlighted-Otter can be screwed onto an empty bottle of Mate (or similar), therefore serving with a very small footprint.

## Images:

![Top View](https://raw.githubusercontent.com/Jan--Henrik/Enlighted-Otter/master/Images/Enlighted_Otter_1.jpeg)
![Side View](https://raw.githubusercontent.com/Jan--Henrik/Enlighted-Otter/master/Images/Enlighted_Otter_2.jpeg)
![Bottom View](https://raw.githubusercontent.com/Jan--Henrik/Enlighted-Otter/master/Images/Enlighted_Otter_3.jpeg)

## Videos

[![Alt text for your video](http://i3.ytimg.com/vi/A-QjU9mWTO4/maxresdefault.jpg)](https://youtu.be/A-QjU9mWTO4)

## Planning

### To do:

- [ ] Test Injected ADC
- [ ] Write regulator
- [ ] Add second TSC bank
- [ ] Write the user interface
- [ ] Fix the HRTIM FLT1 line
- [ ] Find more to do's

### Done:

- [x] Get HRTIM to run
- [x] Get the LED's to light up
- [x] Basic TSC,UART,DAC,COMP,I2C and GPIO functionality
- [x] Readout of touchslider
- [x] Be an otter
- [x] PCB does not burn
