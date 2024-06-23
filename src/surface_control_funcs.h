#include <Arduino.h>
#include <PWMServo.h>

#define RIGHT_WING_SERVO_PIN 2
#define LEFT_WING_SERVO_PIN 3
#define RIGHT_ELEVATOR_SERVO_PIN 4
#define LEFT_ELEVATOR_SERVO_PIN 5

#define MAX_SURFACE_CONTROL_ANGLE 15

#define DEFAULT_SERVO_POS 90

PWMServo rightWing;  // create servo object to control a servo
PWMServo leftWing;  // create servo object to control a ser
PWMServo rightElevator;  // create servo object to control a ser
PWMServo leftElevator;  // create servo object to control a ser

void testAllServoMotors() {

    rightWing.write(DEFAULT_SERVO_POS + MAX_SURFACE_CONTROL_ANGLE);
    leftWing.write(DEFAULT_SERVO_POS + MAX_SURFACE_CONTROL_ANGLE);
    rightElevator.write(DEFAULT_SERVO_POS + MAX_SURFACE_CONTROL_ANGLE);
    leftElevator.write(DEFAULT_SERVO_POS + MAX_SURFACE_CONTROL_ANGLE);

    delay(1500);

    rightWing.write(DEFAULT_SERVO_POS - MAX_SURFACE_CONTROL_ANGLE);
    leftWing.write(DEFAULT_SERVO_POS - MAX_SURFACE_CONTROL_ANGLE);
    rightElevator.write(DEFAULT_SERVO_POS - MAX_SURFACE_CONTROL_ANGLE);
    leftElevator.write(DEFAULT_SERVO_POS - MAX_SURFACE_CONTROL_ANGLE);

    delay(1500);

    rightWing.write(DEFAULT_SERVO_POS);
    leftWing.write(DEFAULT_SERVO_POS);
    rightElevator.write(DEFAULT_SERVO_POS);
    leftElevator.write(DEFAULT_SERVO_POS);

}