#include <Arduino.h>
#include "ms4525do.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HTU21D.h>
#include <math.h>
#include <MechaQMC5883.h>
#include <imu_funcs.h>
#include <display_funcs.h>
#include <air_speed_funcs.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <bmp180_funcs.h>
#include <surface_control_funcs.h>
#include <mode_controls.h>
#include <CircularBuffer.hpp>
#include <IntervalTimer.h>

#define ANKARA_PRESSURE 938 // meters from the sea in Ankara

#define PA_TO_HPA 0.01
#define MB_TO_PA 100

#define GPS_TX_PIN 28
#define GPS_RX_PIN 29
#define GPS_SERIAL Serial7

#define FS_IA6_TX_PIN 16
#define FS_IA6_RX_PIN 17
#define FS_IA6_SERIAL Serial4
#define FS_IA6_SERIAL_BUFFER_SIZE 256

CircularBuffer<uint8_t, FS_IA6_SERIAL_BUFFER_SIZE> radioBuffer;

int sensorsTurn = 0;

TinyGPSPlus gps;
double lat, lng;

MechaQMC5883 qmc;

bfs::Ms4525do airSpeedSensor;
const float avg_diff_pres_offset = 128; // in pascals

HTU21D htu;

double temperature, humidity, delta_pressure, delta_pressure_old = 0, air_density;

#define IBUS_BUFFSIZE 32    
#define IBUS_MAXCHANNELS 6

static uint8_t ibusIndex = 0;
static uint8_t ibus[IBUS_BUFFSIZE] = {0};
static uint16_t radioValues[IBUS_MAXCHANNELS];
IntervalTimer radioControllerTimer;
void radioControllerRead();
void processRadioController();

void setup() {
  Serial.begin(6000000);
  uint32_t startTime = millis();
  while(!Serial && (millis() - startTime < 1500)) ;

  Wire.begin();
  Wire.setClock(400000);

  if (!u8g2.begin()) {
    Serial.println("Unable to initialize U8G2 library!");
    while (true) ;
  }
  u8g2.enableUTF8Print();
  Serial.println("U8g2 is initialized!");
  displayBigMessage("...Initializing...");

  FS_IA6_SERIAL.begin(115200);

  rightWing.attach(RIGHT_WING_SERVO_PIN);
  rightWing.write(DEFAULT_SERVO_POS);

  leftWing.attach(RIGHT_WING_SERVO_PIN);
  leftWing.write(DEFAULT_SERVO_POS);

  rightElevator.attach(RIGHT_WING_SERVO_PIN);
  rightElevator.write(DEFAULT_SERVO_POS);

  leftElevator.attach(RIGHT_WING_SERVO_PIN);
  leftElevator.write(DEFAULT_SERVO_POS);

  testAllServoMotors();

  GPS_SERIAL.begin(9600);
  delay(10);

  uint8_t qmc_sense_rate;
  switch (MAG_SENSE_RATE)
  {
  case 200:
    qmc_sense_rate = ODR_200Hz;
    break;
  case 100:
    qmc_sense_rate = ODR_100Hz;
    break;
  case 50:
    qmc_sense_rate = ODR_50Hz;
    break;
  case 10:
    qmc_sense_rate = ODR_10Hz;
    break;
  
  default:
    qmc_sense_rate = ODR_100Hz;
    Serial.println("ERROR: The sense rate chosen is not suppored in QMC5883!");
    break;
  }

  qmc.init();
  qmc.setMode(Mode_Continuous, qmc_sense_rate, RNG_2G, OSR_512);

  Serial.println("Initialized Magnetometer!");

  SPI.beginTransaction(bmi160_settings);
  if (!BMI160.begin(BMI160GenClass::SPI_MODE,  Wire, BMI160_CS_PIN)) {
    Serial.println("Failed to initialize BMI160!");
    displayError("Failed to initialize BMI160!");
    while(true) {}
  }
  Serial.println("Initialized BMI160!");
  displayBigMessage("...Calibrating IMU...");

  BMI160.setGyroRate(BMI160_SENSE_RATE);
  BMI160.setAccelerometerRate(BMI160_SENSE_RATE);
  BMI160.setGyroRange(GYRO_RANGE);
  BMI160.setAccelerometerRange(ACCL_RANGE);

  BMI160.autoCalibrateGyroOffset();
  BMI160.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  BMI160.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  BMI160.autoCalibrateAccelerometerOffset(Z_AXIS, 1); // Meaning that the Z axis should be pointing to the ground
  SPI.endTransaction();

  delay(100);

  SPI.beginTransaction(bmi160_settings);
  int rawXAcc, rawYAcc, rawZAcc;
  BMI160.readAccelerometer(rawXAcc, rawYAcc, rawZAcc);
  SPI.endTransaction();
  float accX = convertRawAccel(rawXAcc);
  float accY = convertRawAccel(rawYAcc);
  float accZ = convertRawAccel(rawZAcc);
  
  float roll  = rad_to_deg(atan(accY / accZ));
  float pitch = rad_to_deg(atan(-accX / sqrt(accY * accY + accZ * accZ)));

  kalmanRoll.setAngle(roll);
  kalmanPitch.setAngle(pitch);
  gyro_roll = roll;
  gyro_pitch = pitch;

  airSpeedSensor.Config(&Wire, 0x28, 1.0f, -1.0f);
  if (!airSpeedSensor.Begin()) {
    Serial.println("Couldn't find Air speed!");
    displayError("Couldn't find Air speed!");
    while(1){}
  }
  Serial.println("Initialized Airspeed sensor!");

  htu.begin();
  htu.setResolution(HTU21DResolution::RESOLUTION_RH8_T12);
  Serial.println("Initialized HTU21D!");

  if (!bmp.begin()) {
    Serial.println("Couldn't find BMP180!");
    displayError("Couldn't find BMP180!");
    while (1);
  }
  Serial.println("Initialized BMP180!");

  calculateAzimuth();
  firstAzimuth = azimuth;

  displayBigMessage("Finished Initializing");
  delay(2000);

  FS_IA6_SERIAL.clear();
  if (!radioControllerTimer.begin(radioControllerRead, 10000)) {
    Serial.println("Unable to set up a timer for radio controller interrupt!");
    displayError("Unable to set up a timer for radio controller interrupt!");
    while (1);
  } 
  lastRadioPacket = millis();
}

void loop() {
  auto start = millis();
  
  double delat_pressure_sum = 0;
  double last_diff_pres = 0;
  if (uav_mode == UAV_MODES::idle) {
    switch (sensorsTurn) {
      case 0: // GPS
        //Serial.printf("Received %d bytes from GPS\n", GPS_SERIAL.available());
        while (GPS_SERIAL.available() > 0) {
          gps.encode(GPS_SERIAL.read());
        }
        if (gps.location.isUpdated()) {
          lat = gps.location.lat();
          lng = gps.location.lng();
          Serial.print("Latitude: ");
          Serial.println(lat, 6);
          Serial.print("Longitude: ");
          Serial.println(lng, 6);
        }
        break;
    case 1: // Humidity
      if (!htu.measure()) {
        Serial.println("WARNING: Unable to measure the humidity!");
      }
      humidity = htu.getHumidity();
      break;
    case 2: // Temperature & pressure
      calculatePressureAndAltitude();
      temperature = T;
      air_density = densityhumidair(pressure_mb * MB_TO_PA, temperature, humidity / 100);
      break;
    case 3: // Airspeed sensor & calculation
      for (int i = 0 ; i < 10 ; i++) {
        if (airSpeedSensor.Read()) {
          last_diff_pres = airSpeedSensor.pres_pa();
        }
        delat_pressure_sum += last_diff_pres;
      }
      delta_pressure = low_pass_filter(delta_pressure_old, delat_pressure_sum / 10 + avg_diff_pres_offset);
      delta_pressure_old = delta_pressure;
      if (delta_pressure > 0) {
        delta_pressure = 0;
      }
      Serial.printf("%.2f delta pres, Humidity: %.2f, Temperature: %.2f, Pressure: %.2f, Air Density: %.3f kg/m^3, air speed: %.2f m/s, altitude: %.2f\n",
       delta_pressure, humidity, temperature, pressure_mb * MB_TO_PA, air_density, pow((2 * abs(delta_pressure)) / air_density, 0.5), altitude);
      break;
    case 4: // Display update & azimuth calculation
      calculateAzimuth();
      displaySensorData(temperature, humidity, pressure_mb * MB_TO_PA, azimuth, true, lat, lng);
      break;
    }
  } else if (uav_mode == UAV_MODES::vtol) {
    ;
  } else if (uav_mode == UAV_MODES::fixed_wing) {
    ;
  } else if (uav_mode == UAV_MODES::failsafe) {
    // do nothing!
  }
  sensorsTurn++;
  if (sensorsTurn == 7) {
    sensorsTurn = 0;
  }

  calculateRollPitch();

  Serial.print(millis() - start);
  Serial.println(" ms");

  processRadioController();
}

void radioControllerRead() {
  while (FS_IA6_SERIAL.available()) {
    radioBuffer.push(FS_IA6_SERIAL.read());
  }
}

void processRadioController() {
  while (radioBuffer.size() >= IBUS_BUFFSIZE) {
    uint8_t val = radioBuffer.shift();
    if (ibusIndex == 0 && val != 0x20) {
      ibusIndex = 0;
      break;
    }
    if (ibusIndex == 1 && val != 0x40) {
      ibusIndex = 0;
      break;
    }

    if (ibusIndex == IBUS_BUFFSIZE) {
      ibusIndex = 0;
      int high=3;
      int low=2;
      for(int i=0;i<IBUS_MAXCHANNELS; i++) {
        radioValues[i] = (ibus[high] << 8) + ibus[low];
        high += 2;
        low += 2;
      }
      lastRadioPacket = millis();
      Serial.printf("%d, %d, %d, %d, %d, %d\n", radioValues[0], radioValues[1], radioValues[2], radioValues[3] ,radioValues[4], radioValues[5]);
    }
    else {
      ibus[ibusIndex] = val;
      ibusIndex++;
    }
  }
  if (!isRadioRunning(radioValues)) {
    switchToFailSafe();
  } else {
    exitFailSafeMode();
  }
}
