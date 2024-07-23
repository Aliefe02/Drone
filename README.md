# Drone
Drone controlling programs to control a drone using Raspberry Pi

# Ground Station
Draw a path on the desktop application and set parameters, hit Start Drone. Drone will take off and follow that path according to your parameters.

**** Path Follow Object Avoidence
Uses same communication as Ground Station, however this program uses 3 ultrasonic sensors to measure distance of front, left and right. If any object is detected, it runs an algorithm to avoid it.


# Keyboard Controller
Control your drone using keyboard of your pc. Key bindings are written in the code.

Requirements<br/>
---------------
Flight controller with ArduPilot Software<br/>
Raspberry Pi with necessary libraries installed for drone connection<br/>
0.96 inch OLED display connected to RPi to display data<br/>
2 LoRa devices (I use ra-02)<br/>
Arduino Nano <br/>

Connect 1 LoRa to RPi and other to Nano. Connect Nano to laptop to use serial connection
