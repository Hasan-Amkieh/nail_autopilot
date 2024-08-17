An autopilot was designed from scratch for a transwing UAV, equipped with many sensors, including, air speed sensor, atmospheric sensors, GPS, LCD display, 2x E32 LoRa, and a stepper motor driver.

# Hardware
Teensy 4.1 was used as the main microprocessor autopilot, responsible for establishing communcation with the control station through the user interface (E32 Lora), performing radio controls, and much more. 

![autopilot circuit front](https://github.com/user-attachments/assets/184d9f84-9782-4d1c-bb1d-8b5617900fa9)
![autopilot circuit back](https://github.com/user-attachments/assets/adfbab71-cf4f-494a-91c8-81a7f939479d)
![autopilot circuit](https://github.com/user-attachments/assets/75558e4f-dd30-42d1-be1d-dbeeeceb7223)

# Software
Partial DrehmFlight 1.3 was used to aid the software development, mostly used in PID-related code.

