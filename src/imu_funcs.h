#pragma once
#include <BMI160Gen.h>
#include <Kalman.h>
#include <SPI.h>

#define ACCL_RANGE     2
#define GYRO_RANGE   250
#define MAG_SENSE_RATE   50
#define BMI160_SENSE_RATE 200.0

#define deg_to_rad(a) (a/180*M_PI)
#define rad_to_deg(a) (a/M_PI*180)

#define ANKARA_DECLINATION 1.18333333333333333333333333 // east is positive

#define SENITIVITY_2G 120.0 // 120.0 LSB / MicroTesla == 12000.0 LSB / Gauss

#define BMI160_CS_PIN 10

#define B_GYRO 0.1

const SPISettings bmi160_settings = SPISettings(1000000, MSBFIRST, SPI_MODE0);

float omega_roll  = 0;
float omega_pitch = 0;
float omega_yaw   = 0;

float omega_roll_prev  = 0;
float omega_pitch_prev = 0;
float omega_yaw_prev   = 0;

static float gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
static uint32_t last_micros = 0;

Kalman kalmanRoll;
Kalman kalmanPitch;

float kalm_roll;
float kalm_pitch;

float firstAzimuth = 0;
float yaw;

float azimuth = 0, old_azimuth;
float mag_x_hor = 0, mag_y_hor = 0;

const double hard_iron[3] = {
  0.8049999999999997, 10.829999999999998, -4.18
};

float low_pass_filter(float old_val, float new_val) {
  return old_val * 0.6 + new_val * 0.4;
}

inline float convertRawGyro(int gRaw) {
  // ex) if the range is +/-500 deg/s: +/-32768/500 = +/-65.536 LSB/(deg/s)
  float lsb_omega = float(0x7FFF) / GYRO_RANGE;
  return gRaw / lsb_omega;  // deg/sec
}

inline float convertRawAccel(int aRaw) {
  // ex) if the range is +/-2g ; +/-32768/2 = +/-16384 LSB/g
  float lsb_g = float(0x7FFF) / ACCL_RANGE;
  return aRaw / lsb_g;
}

void calculateRollPitch() {
  
  // read raw accl measurements from device
  int rawXAcc, rawYAcc, rawZAcc; // x, y, z
  SPI.beginTransaction(bmi160_settings);
  BMI160.readAccelerometer(rawXAcc, rawYAcc, rawZAcc);
  SPI.endTransaction();
  float accX = convertRawAccel(rawXAcc);
  float accY = convertRawAccel(rawYAcc);
  float accZ = convertRawAccel(rawZAcc);

  float rad_a_roll = atan2(accY, accZ);
  float rad_a_pitch = atan2(-accX, sqrt(accY*accY + accZ*accZ));

  float accl_roll = rad_to_deg(rad_a_roll);
  float accl_pitch = rad_to_deg(rad_a_pitch);

  int rawRoll, rawPitch, rawYaw;
  SPI.beginTransaction(bmi160_settings);
  BMI160.readGyro(rawRoll, rawPitch, rawYaw);
  SPI.endTransaction();
  omega_roll  = convertRawGyro(rawRoll);
  omega_pitch = convertRawGyro(rawPitch);
  omega_yaw   = convertRawGyro(rawYaw);
  
  // Lowpass filter:
  omega_roll = (1.0 - B_GYRO) * omega_roll_prev + B_GYRO * omega_roll;
  omega_pitch = (1.0 - B_GYRO) * omega_pitch_prev + B_GYRO * omega_pitch;
  omega_yaw = (1.0 - B_GYRO) * omega_yaw_prev + B_GYRO * omega_yaw;

  omega_roll_prev = omega_roll;
  omega_pitch_prev = omega_pitch;
  omega_yaw_prev = omega_yaw;
  
  uint32_t cur_micros = micros();
  uint32_t duration = cur_micros - last_micros;
  last_micros = cur_micros;
  double dt = duration / 1000000.0; // us->s
  //if (duration > 50000) return; // if more than 50 ms has passed, then the gyro reading becomes bad, thus it must be restarted

  gyro_roll  += omega_roll  * dt; 
  gyro_pitch += omega_pitch * dt;
  gyro_yaw   += omega_yaw   * dt;
  
  kalm_roll  = kalmanRoll.getAngle(accl_roll, omega_roll, dt);
  kalm_pitch = kalmanPitch.getAngle(accl_pitch, omega_pitch, dt);

  //Serial.printf("%2f,%2f,%2f\n", accX, gyro_roll, accY);

}

void calculateAzimuth() {

  int16_t raw_x = 0, raw_y = 0, raw_z = 0;

  Wire.beginTransmission(0x0D);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.requestFrom(0x0D, 6);
  raw_x = Wire.read(); // LSB  x
  raw_x |= Wire.read() << 8; // MSB  x
  raw_y = Wire.read(); // LSB  z
  raw_y |= Wire.read() << 8; // MSB z
  raw_z = Wire.read(); // LSB y
  raw_z |= Wire.read() << 8; // MSB y

  float x = raw_x / SENITIVITY_2G;
  float y = raw_y / SENITIVITY_2G;
  float z = raw_z / SENITIVITY_2G;

  double mag_data[3] = {x, y, z};
  for (uint8_t i = 0 ; i < 3 ; i++) {
    mag_data[i] = mag_data[i] - hard_iron[i];
  }

  float mag_pitch = kalm_roll * DEG_TO_RAD;
  float mag_roll = -kalm_pitch * DEG_TO_RAD;

  mag_x_hor = mag_data[0] * cos(mag_pitch) + mag_data[1] * sin(mag_roll) * sin(mag_pitch) - mag_data[2] * cos(mag_roll) * sin(mag_pitch);
  mag_y_hor = mag_data[1] * cos(mag_roll) + mag_data[2] * sin(mag_roll);

  old_azimuth = azimuth;
  azimuth = rad_to_deg(atan2(mag_x_hor, mag_y_hor));

  azimuth += ANKARA_DECLINATION + 90.0;
  azimuth = azimuth < 0 ? 360 + azimuth : azimuth;

  yaw = firstAzimuth - azimuth;
  if (yaw > 360.0) {
    yaw -= 360.0;
  }
  if (yaw < 0.0) {
    yaw += 360.0;
  }

  //Serial.printf("Uni:0.00,0.00,0.00,0.00,0.00,0.00,%.2f,%.2f,%.2f\n", x, y, z);

}
