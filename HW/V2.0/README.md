### Changelog HW 2.0:

- Moved all parts to the bottom side
- Changed battery clips to SMD
- Added fuses
- Removed GND-Plane around the Touch-Pads
- Better power plane splitting
- Changed ESP footprint
- Added moodlight leds

Assembled HW 2.0, seems to be much sturdier, everything looking good. GND-Plane is broken, NTC return GND is broken. Tocuh performance is much better, analog performance seems to be much better (needs to measured). Current goes now up to 499mA

### Todo list

- [ ] SW 2.0
	- [x] fix i2c
	- [x] take measurements, configure EO
		- looks good so far
	- [x] fix auto turnoff
	- [ ] No current > 499mA (EO will turn off)
	- [ ] Test LEDS and ESP
		- [x] LEDS
		- [ ] ESP
	- [x] fix touch direction and intensity


- [x] HW 2.0
	- [x] move 254TR closer together
	- [x] move parts to the bottom side
	- [x] add esd protection resistors
		- touch
		- ntc
	- [x] fix usb plug
	- [x] Optimize values in circuit
	- [x] fix touch
		- round planes, remove ground
		- Needs to be tested
	- [ ] Fix GND-Plane
	- [ ] FIX values as in "Noted"
	- [ ] FIX NTC-GND
