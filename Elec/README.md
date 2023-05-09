#Electronics overview
I use these parts:
1. Temperature sensors -- as indicated by the name "RaceTemp":  NTC for water temp. and MAX6675+Thermocouple for EGT  
2. Controller board: "WeAct Black Pill" with STM32F4 and a ST-link debugger.  
You can see the MCU pinout I use in STM32CubeMX (open the RaceTemp.ioc file).  
I have also used Arduino Nano and "Blue Pill" STM32F1.  Maybe I will also try ESP32 S3 and BLE  
4. WiFi module: ESP01 -- for connection to the phone.   (I have also used a HC05-module and Bluetooth SPP. The code is still mostly compatible with HC05, but the ESP01 is better)  
5. Sat.Nav. (GPS) module: uBlox M9N (at 10 Hz) and a ceramic patch antenna -- for lap/split times, speed and acceleration
6. Ignition probe for engine RPM.  This is just a wire and some protection diodes connected to one of the microcontroller's timer inputs     
7. Gyro/accelerometer: MPU9250. This worked OK until I disconnected and disabled it because "g" anyway will be calculated based on Sat.Nav.  

You can of course add your own selection of other sensors -- see for example the pedal sensor in the Mech folder.  My plan:
1) $\lambda$ or O<sub>2</sub>: Bosch LSU-ADV and Bylund's LambdaShield with Bosch CJ125 chip. 
The code for this is included and enabled -- just hook it up and try!
I prefer to present the results as "Fuel Excess" -- negative FuEx means lean (danger), positive FuEx is rich (safe).  FuEx = 1 / $\lambda$ - 1  
2) Knock sensor: TPIC8101DWTR chip and 0261231176 sensor.  I did not yet write any code for this. 

#Sat.Nav. receiver
Ublox Neo M9N module

#Sat.Nav. antenna  
Ceramic patch antena, size 25x25 mm, active (w/amplifier), 28 dBi,  10 mA 3.3 to 5 V.  
Quote from ref. [1]: "There is no need for the antenna LNA gain to exceed 26 dB for use with u-blox receivers (at the RF input). With shorter
cables and a gain above 35 dB, an overload condition might occur on some receivers."

Ceramic patch antena, size 25x25 mm, connected via a short (30 mm) coax to the Sat.Nav. receiver.  
The antenna should have the ceramic side up and little obstructions towards the sky, and the metallic side down towards a ground plane.
I.e.: No cables or circuit boards (and little of other stuff) above the antenna.

The antenna can be placed inside the electronics box (if using a plastic box and the orientation is correct). 

#Electromagnetic noise:  
The engine ignition (on my go kart) causes a lot of electromagnetic noise.  
Therefore: All wires entering the box should (at entry) have both a 100 nF connected to GND and a TVS diode (also to GND).  
This will both protect the electronics and also increase accuracy.  

List of references:  
Ref. 1:  uBlox GNSS-Antennas_AppNote UBX-15030289 - R03.  
