# RaceTemp
Software, electronics (input devices) and mechanical accessories for using a phone as karting logger and display.  
The phone can show not only temperatures (water and exhaust gas), but also lap/sector times from sat.nav. (GPS) 
and other sensors like ignition probe (for engine RPM), accelerometer, gyro, engine knock, lambda, throttle, 
brake, etc. etc.  The recorded data can be reviewed later and you may also overlay the sensors onto your recorded 
video (e.g. from GoPro or phone).  You will need a phone with RaceChrono or TrackAddict, and you will need to 
make your own input device.  
[Link to video with sensor overlay](https://fb.watch/k7wkuMYCiB/)

# "Arduino" folder 
Short simple Arduino projects can be a good starting point if you have little 
programming experience or limited patience. I have tested these only with ESP32s3. 

# "Mech" folder 
Mechanical stuff: Electronics box, Battery holder, CAD models of the 3D-printed bracket and bezel I use for attaching my Oscal S80 phone to a Sodikart steering wheel -- they will probably not fit your phone or your steering wheel, but you are free to modify. If you make a pull request (or send your models to me), then I can also add your CAD models to the project.  

# "Elec" folder 
Electrical stuff: Information about sensors and circuits used.     

# "Core" folder 
Software, c-source code.  Auto-generated code from CubeMX also ends up in this folder, but will not 
be checked in unless one change the .gitignore files or specifically adds the file to git.

# How to get started with software for STM32: 
1. Install STM32CubeIDE and STM32CubeMX on a PC  
2. Open the RaceTemp.ioc file in CubeMX to view (or edit) pinout or other settings.  Then click "GENERATE CODE" and wait a few seconds for the popup "The Code is successfully..." with a blue "Open Project" button. (If that button does not start CubeIDE, then you need to find another way to start CubeIDE and go File-->Open project from file system-->select the RaceTemp folder-->Finish )  
3. In CubeIDE you edit whatever you need in the source code.  You should at least:  
    a) Change the network SSID and password in Core/Inc/pass.h (unless you are comfortable with "MySSID" and "MyPassword"), and  
    b) Enable "use float with printf...".  Right-click on RaceTemp in the left pane (project tree), select C/C++Build --> Settings --> MCU Settings --> Check the box for "Use float printf..." --> Apply and Close  
4. When the project compiles OK, then connect your debugger and download to the microcontroller.  It is also possible to download via USB (e.g. from Arduino or STM32CubeProgrammer), but seriously: Get a debugger!  STlink v2 clones are almost free...   
5. Connect your phone via WiFi to "MySSID" (or whatever you called it, see step #3a).  Start/install "Serial WiFi Terminal" on your phone to view the messages    
6. Tasks and scheduling is handled via FreeRTOS.  Four OS tasks are defined via CubeMX (and generated into Core/Src/freertos.c):  
   a) DefaultTsk --> the idle task  
   b) LambdaTsk --> lambda_2.c -- interface to a Bosch LSU-ADV O2-sensor via a Bosch CJ125 ASIC  
   c) RaceTempTsk --> RaceTemp.c -- sensor inputs and serial output to RaceChrono or TrackAddict (via a wireless serial interface)  
   d) UBX_task --> UBX.c -- serial IO to/from GNS receiver via USART and DMA  
