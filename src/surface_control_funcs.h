#include <Arduino.h>
#include <PWMServo.h>
#include <imu_funcs.h>

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

#define MAX_ROLL_VTOL 20.0
#define MAX_PITCH_VTOL 20.0
#define MAX_YAW_VTOL 10.0
#define I_LIMIT 15.0  // Integrator saturation level, mostly for safety (default 25.0)

#define IBUS_BUFFSIZE 32    
#define IBUS_MAXCHANNELS 6

static uint16_t radioValues[IBUS_MAXCHANNELS] = {2000};
static uint8_t ibusIndex = 0;
static uint8_t ibus[IBUS_BUFFSIZE] = {0};
IntervalTimer radioControllerTimer;
void radioControllerRead();
void processRadioController();

PWMServo rightWing;
PWMServo leftWing;
PWMServo rightElevator;
PWMServo leftElevator;

PWMServo rightFirstM;
PWMServo rightLastM;
PWMServo leftFirstM;
PWMServo leftLastM;

/*
ranges between 0 and 180
0 - first right
1 - last right
2 - first left
3 - last left
*/
uint16_t motor_throttles[4] = {0};

/*
ranges between 0 and 180
0 - right aileron
1 - left aileron
2 - right elevator
3 - left elevator
*/
uint16_t surface_controls[4] = {0};

float throttle_des = 0.0, roll_des = 0.0, pitch_des = 0.0, yaw_des = 0.0;
float roll_passthru = 0.0, pitch_passthru = 0.0, yaw_passthru = 0.0;
float dt = 0.0, current_time = 0.0, prev_time = 0.0;

float error_roll = 0.0, integral_roll = 0.0, integral_roll_prev = 0.0, derivative_roll = 0.0, roll_PID = 0.0;
float error_pitch = 0.0, integral_pitch = 0.0, integral_pitch_prev = 0.0, derivative_pitch = 0.0, pitch_PID = 0.0;

float Kp_roll_angle = 0.2;    // Roll P-gain
float Ki_roll_angle = 0.35;    // Roll I-gain
float Kd_roll_angle = 0.05;   // Roll D-gain

float Kp_pitch_angle = 0.2;   // Pitch P-gain
float Ki_pitch_angle = 0.35;   // Pitch I-gain
float Kd_pitch_angle = 0.05;  // Pitch D-gain

float Kp_yaw = 0.0;           // Yaw P-gain
float Ki_yaw = 0.0;          // Yaw I-gain
float Kd_yaw = 0.0;       // Yaw D-gain

/*
float Kp_yaw = 0.3;    
float Ki_yaw = 0.05;   
float Kd_yaw = 0.00015;
*/

void commandMotors() {
    rightFirstM.write(motor_throttles[0]);
    rightLastM.write(motor_throttles[1]);
    leftFirstM.write(motor_throttles[2]);
    leftLastM.write(motor_throttles[3]);
}

void commandSurfaceControls() {
    rightWing.write(surface_controls[0]);
    leftWing.write(surface_controls[1]);
    rightElevator.write(surface_controls[2]);
    leftElevator.write(surface_controls[3]);
}

void testAllServos() {

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

void getDesStateVTOL() {
  throttle_des = (radioValues[2] - 1000.0) / 1000.0; // Between 0 and 1
  roll_des = (radioValues[3] - 1500.0) / 500.0; // Between -1 and 1
  pitch_des = (radioValues[1] - 1500.0) / 500.0; // Between -1 and 1
  yaw_des = (radioValues[0] - 1500.0) / 500.0; // Between -1 and 1
  roll_passthru = roll_des / 2.0; // Between -0.5 and 0.5
  pitch_passthru = pitch_des / 2.0; // Between -0.5 and 0.5
  yaw_passthru = yaw_des / 2.0; // Between -0.5 and 0.5
  
  //Constrain within normalized bounds
  throttle_des = constrain(throttle_des, 0.0, 1.0); //Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0) * MAX_ROLL_VTOL; //Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0) * MAX_PITCH_VTOL; //Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0) * MAX_YAW_VTOL; //Between -maxYaw and +maxYaw
  roll_passthru = constrain(roll_passthru, -0.5, 0.5);
  pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
  yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
}

void controlAngleVTOL() {
  
  //Roll
  error_roll = roll_des - kalm_roll;
  integral_roll = integral_roll_prev + error_roll * dt;
  if (radioValues[2] < 1060) {   // Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -I_LIMIT, I_LIMIT); // Saturate integrator to prevent unsafe buildup
  derivative_roll = omega_roll;
  roll_PID = 0.01 * (Kp_roll_angle * error_roll + Ki_roll_angle * integral_roll - Kd_roll_angle * derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - kalm_pitch;
  integral_pitch = integral_pitch_prev + error_pitch * dt;
  if (radioValues[2] < 1060) {   // Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -I_LIMIT, I_LIMIT); // Saturate integrator to prevent unsafe buildup
  derivative_pitch = omega_pitch;
  pitch_PID = .01 * (Kp_pitch_angle * error_pitch + Ki_pitch_angle * integral_pitch - Kd_pitch_angle * derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  // Yaw, stablize on rate from GyroZ
  /*error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -I_LIMIT, I_LIMIT); // Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range
  */

  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  //error_yaw_prev = error_yaw;
  //integral_yaw_prev = integral_yaw;
}

void controlMixerVTOL() {
   
  motor_throttles[2] = (throttle_des - pitch_PID + roll_PID) * 180; // first left
  motor_throttles[0] = (throttle_des - pitch_PID - roll_PID) * 180; // first right
  motor_throttles[1] = (throttle_des + pitch_PID - roll_PID) * 180; // last right
  motor_throttles[3] = (throttle_des + pitch_PID + roll_PID) * 180; // last left

  /*
  m1_command_scaled = throttle_des - pitch_PID + roll_PID + yaw_PID; // first left
  m2_command_scaled = throttle_des - pitch_PID - roll_PID - yaw_PID; // first right
  m3_command_scaled = throttle_des + pitch_PID - roll_PID + yaw_PID; // last right
  m4_command_scaled = throttle_des + pitch_PID + roll_PID - yaw_PID; // last left
  */
 
}
