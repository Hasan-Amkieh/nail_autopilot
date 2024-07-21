#include <Arduino.h>
#include <PWMServo.h>

#define RIGHT_WING_SERVO_PIN 2
#define LEFT_WING_SERVO_PIN 3
#define RIGHT_ELEVATOR_SERVO_PIN 4
#define LEFT_ELEVATOR_SERVO_PIN 5

#define RIGHT_FIRST_MOTOR_PIN 14
#define RIGHT_LAST_MOTOR_PIN 15
#define LEFT_FIRST_MOTOR_PIN 23
#define LEFT_LAST_MOTOR_PIN 22

#define MAX_SURFACE_CONTROL_ANGLE 15

#define DEFAULT_SERVO_POS 90

PWMServo rightWing;
PWMServo leftWing;
PWMServo rightElevator;
PWMServo leftElevator;

PWMServo rightFirstM;
PWMServo rightLastM;
PWMServo leftFirstM;
PWMServo leftLastM;

/*
0 - first right
1 - last right
2 - first left
3 - last left
*/
uint16_t motor_throttles[4] = {0}; // ranges between 0 and 180

/*
0 - right aileron
1 - left aileron
2 - right elevator
3 - left elevator
*/
uint16_t surface_controls[4] = {0}; // ranges between 0 and 180

void throttleUpdate() {
    rightFirstM.write(motor_throttles[0]);
    rightLastM.write(motor_throttles[1]);
    leftFirstM.write(motor_throttles[2]);
    leftLastM.write(motor_throttles[3]);
}

void surfaceControlsUpdate() {
    rightWing.write(surface_controls[0]);
    leftWing.write(surface_controls[1]);
    rightElevator.write(surface_controls[2]);
    leftElevator.write(surface_controls[3]);
}

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