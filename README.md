An autopilot was designed from scratch for a transwing UAV, equipped with many sensors, including, air speed sensor, atmospheric sensors, GPS, LCD display, 2x E32 LoRa, and a stepper motor driver.

# Hardware
Teensy 4.1 was used as the main microprocessor autopilot, responsible for establishing communcation with the control station through the user interface (E32 Lora), performing radio controls, and much more. 

![autopilot circuit front](https://github.com/user-attachments/assets/184d9f84-9782-4d1c-bb1d-8b5617900fa9)
![autopilot circuit back](https://github.com/user-attachments/assets/adfbab71-cf4f-494a-91c8-81a7f939479d)
![autopilot circuit](https://github.com/user-attachments/assets/75558e4f-dd30-42d1-be1d-dbeeeceb7223)

# Software
Partial DrehmFlight 1.3 was used to aid the software development, mostly used in PID-related code. It has many modes, idle, vtol, transitioning, fixedwing, and failsafe. Idle mode is the default mode for the UAV. The failsafe mode can be triggered for one of these reasons: 
* Radio communication cut-off
* Control station (E32 Lora) cut-off
* Sensors communication-related error
* Manually activating fail-safe from control station

Here is a breakdown of all the sensors used:

* MS4525 airspeed sensor
* SSD1306 LCD display (displaying errors, failsafe mode cause, sensors values if idle mode)
* BMI160 6-DOF accelerometer & gyroscope
* QMC5883 magnetometer
* BMP180 pressure sensor
* HTU21DF humidity sensor
* 2x E32 Lora modules, a transmitter and a receiver, increases bidirectional communication speed
* Neo-M8N GPS module
* Bidirectional Channel Shifter (shifts the voltage from 3.3 to 5 to control the stepper motor driver)
* DM556 stepper motor driver (controls the wings position for the transitioning functionality) 
