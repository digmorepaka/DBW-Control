# Drive by wire controller

## For bench use, testing use, stationary engine use, offroad use, and completely without warranty or liability from me or any of the contributors

## Features:

* UI over Serial
* PID Closed loop control
* Configurable TPS and APP calibration, stored in local EEPROM
* Configurable 3 point Non-linear APP to TPS relation
* EEPROM intergrity checksum
* Multiple throttle states(idle, transiet, stable, safety) to improve behavior
* Non-linear 3 point TPS and APP position sensor failure detection


## TODO:
* Automatic TPS1/TPS2 and APP1/APP2 failsafe correlation calibration
* Failsafe mode that actually works
* Increase TPS vs APP configurable cell count
* Throttle smoothing/delay
* Design HW
* Get to understand parts of the code that I don't undestand :P
* Improve UI, possibly integrate with TunerStudio
* Code cleanup and optimization

## Dependencies:
* frankboesing/FastCRC
* br3ttb/PID
* greygnome/EnableInterrupt
* robtillaart/MultiMap

## Hardware
Currently testing with an Arduino UNO and a H bridge. H bridge is configured with a TC4424 driver and a NOT gate as to have a **single direction select pin, and a 5th enable/disable MOSFET**. TC4425/4428 is a better choice because it doesn't needan inverter due to it's mix of inverting/non inverting drivers. This is what I had on hand.

Currently testing with a **Toyota DENSO 89281-52021 Pedal and 22030-0D010 ThrottleBody** from a 2004-2009 Corolla Verso 1ZZ-FE. 2002+ MR2/Celica uses something similar. In the future I will also be working with Kawasaki Versys 1000 2015+ Individual **Mikuni 6307120 throttlebodies**.
