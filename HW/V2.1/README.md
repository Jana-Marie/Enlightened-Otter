### Assemby instructions

Everything can be assembled as per manufacturer except for the following parts.

 - LED1,LED2,LED3 are per BOM sidemounted LEDs but they have to be rotated by 90 degrees and treated as reverse mount leds.
 - The studs cannot stand by themselfes, thus they have to be supported while reflowing
 - D7,D8 are normal 0805 LEDs but also reverse mounted
 
 - After reflowing the batteries can be inserted, EO should boot up and the middle led should light up. EO can now be flashed via Tag-connect
 - before adding the LED pcb it is mandatory to turn off EO!
 - Insulating washers (TO220 isolators) have to be added to all studs except for the GND stud, a non isolating washer has to be addded here
 - Use conducting screws and don't tightem them too much, try to reduce stress introduced into the pcb

### Changelog HW 2.11:

- re-routed everything, this time with good power design
- made pads for studs larger in diameter
- switched to kicad

![V2.1](/Images/v21_back_1.jpg)

### Todo list

- [x] SW 2.1
- [ ] SW 2.11
- [ ] Test LEDS and ESP
  - [x] LEDS
  - [ ] ESP
