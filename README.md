# RaceTemp
This repository contains software, electronics (input devices) and mechanical accessories for using a phone as karting logger and display.  You will need a phone with RachChrono or TrackAddict.  The phone can show not only temperatures (water and exhaust gas), but also lap/sector times from sat.nav. (GPS) and other sensors like ignition probe (for engine RPM), accelerometer, gyro, engine knock, lambda, throttle, brake, etc. etc.  The recorded data can be reviewed later and you may also overlay the sensors onto your recorded video (e.g. from GoPro or phone).   [Link to video with sensor overlay](https://fb.watch/k7wkuMYCiB/)

# "Mech" folder 
CAD models of the 3D-printed bracket and bezel I use for attaching my Oscal S80 phone to a Sodikart steering wheel -- they will probably not fit your phone or your steering wheel, but you are free to modify. If you make a pull request (or send your models to me), then I can also add your CAD models to the project.  

# "Elec" folder 
This folder is empty (or not yet checked into Github), but I use these parts: 
1. Temperature sensors -- as indicated by the name "RaceTemp":  NTC for water temp. and MAX6675+Thermocouple for EGT  
2. Controller board: "WeAct Black Pill" with STM32F4 and a ST-link debugger.  
You can see the MCU pinout I use in STM32CubeMX (open the RaceTemp.ioc file).  
I have also used Arduino Nano and "Blue Pill" STM32F1.  Maybe I will also try ESP32 S3 and BLE  
4. WiFi module: ESP01 -- for connection to the phone.  I used Bluetooth SPP, HC05 earlier and the code is still mostly compatible with this  
5. Sat.Nav. (GPS) module: uBlox M9N (at 10 Hz) -- for lap/split times, speed and acceleration
6. Ignition probe for engine RPM.  This is just a wire and some protection diodes connected to one of the microcontroller's timer inputs     
7. Gyro/accelerometer: MPU9250. This worked OK until I disconnected and disabled it because "g" anyway will be calculated based on Sat.Nav.  

You can of course add your own selection of other sensors.  My plan:
1) $\lambda$ or O<sub>2</sub>: Bosch LSU-ADV and Bylund's LambdaShield with Bosch CJ125 chip.    The code for this is included and enabled -- just hook it up and try!  
   I prefer to present the results as "Fuel Excess" -- negative FuEx means lean (danger), positive FuEx is rich (safe).  FuEx = 1 / $\lambda$ - 1  
2) Knock sensor: TPIC8101DWTR chip and 0261231176 sensor   

# How to get started with software for STM32: 
1. Install STM32CubeIDE and STM32CubeMX on a PC  
2. Open the RaceTemp.ioc file in CubeMX to view (or edit) pinout or other settings.  Then click "GENERATE CODE" and wait a few seconds for the popup "The Code is successfully..." with a blue "Open Project" button  
3. "Open Project" will open the RaceTemp project in CubeIDE.  Here you edit whatever you need in the source code.  You should at least:  
    a) Change the network SSID and password in Core/Src/RaceTemp.c, and  
    b) Enable "use float with printf...".  Right-click on RaceTemp in the left pane (project tree), select C/C++Build --> Settings --> MCU Settings --> Check the box for "Use float printf..." --> Apply and Close  
4. When the project compiles OK, then connect your debugger and download to the microcontroller    
5. Tasks and scheduling is handled via FreeRTOS.  Four OS tasks are defined via CubeMX (and generated into Core/Src/freertos.c):  
   a) DefaultTsk --> the idle task  
   b) LambdaTsk --> lambda_2.c -- interface to a Bosch LSU-ADV O2-sensor via a Bosch CJ125 ASIC  
   c) RaceTempTsk --> RaceTemp.c -- sensor inputs and serial output to RaceChrono or TrackAddict (via a wireless serial interface)  
   d) UBX_task --> UBX.c -- serial IO to/from GNS receiver via USART and DMA  
