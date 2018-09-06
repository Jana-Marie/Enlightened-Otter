### Changelog HW 1.1:

- Fixed tha battery clip position
- Gate drivers are now powered by the boost converters, thus they are bootstrapping themselfes
- VIN measurement is not needed anymore, hardware removed
- Added mosfet to power rail of ESP
- Fixed USB Footprint
- Added pullup to NTC
- Added Gate pulldowns
- Added USB pulldowns
- Fixed Schematic symbols and values
- Increased Gate resistors
- Added Bootstrap diodes



### Todo list

- [ ] HW 1.1 
	- [ ] Optimize values in circuit
	- most 47k to 33k
	- there are too many different capacitor values

- [ ] HW 1.2


- [x] HW 1.1
	- [x] Add bootstrap diodes to start the gate drivers
	- [x] Increase Gate resistor to at least 56 Ohm and maybe add gate capacitor
	- [x] Move 18650 clips to fit battery
	- dont fit the battery, move by 1-2mm
	- [x] Add pullup to NTC (LED)
	- 10k
	- [x] Check USB footprint
	- Part didnt fit footprint that well, already changed but not tested!
	- [x] Remove VIN measurement
	- not needed anymore
	- [x] Add CC1,CC2 pulldown resistors
	- Had only one and CC1/2 connected together, ofc that didn't work
	- [x] Check for analog design improvements
	- Analog design seems to be okay by now, will check later again :3
	- pushed something here and there, i guess its fine now
	- [x] Gate pulldowns
	- Gate driver has no pulldowns, unknown behaviour while not driven
	- [x] Reduce Boost inductance
	- 15uH is too high, 3.3uH is better, also increase dc-current, maybe use this inductor https://de.rs-online.com/web/p/drahtgewickelte-smd-induktivitaten/9236194/
	- [x] Change voltage regulator to 3.0V type
	- Is still 3.3V type in schematic, 3.0V was used for testing
	- [x] Divide Vin voltage divider resistance by 10
	- ~~Resistance seems to be too high, measuring falsifies the result~~ measured to fast, meh! Is fixed now
	- [x] Add Mosfet to turn ESP off
	- ESP will be turned on all the time, add sleepmode and/or switched power 
	- [x] Check schematic symbols
	- Mosfets and inductors have the wrong part numbers (mosfet has to be DMN3404L-7, inductor to be determined)
	- [ ] Export PCB design files to kicad and publish
	- Also export Gerber?
	- [x] Fix OTG mode
		- ~~Device enters not automatically OTG mode, can be done in software or with hardware IO. Problem: OTG/USB voltage is needed for the mosfet drivers. Thus USB has to be rpesent or OTG has 
to be turned on. OTG being turned on means, that on USB power it can't know that it has to move abck to charger mode. Idea 1: use a diode behind the USB plug drive OTG mode from IO and set IO with VUSB 
-> meh Idea 2: move Vgatedriver to VMID, VMID has OTG capabilities~~
		- VSupply of the Gate driver aro to be connected to the outputs of the boost converters. Circuitry gets more effizient when boost converter is running
		- maybe check if there is a cheaper battery management IC as OTG is not needed anymore
