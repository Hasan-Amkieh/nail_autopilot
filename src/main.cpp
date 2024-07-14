#include <Arduino.h>
#include "ms4525do.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <sensors/HTU21D.h>
#include <math.h>
#include <sensors/MechaQMC5883.h>
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
#include <SD.h>
#include <iostream>
#include <sstream>
#include <string>
#include <lora_comm.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstring>

#define ANKARA_PRESSURE 938 // meters from the sea in Ankara

#define PA_TO_HPA 0.01
#define MB_TO_PA 100

#define BATTERY_VOLTAGE_PIN 41

#define GPS_TX_PIN 28
#define GPS_RX_PIN 29
#define GPS_SERIAL Serial7

#define FS_IA6_TX_PIN 16
#define FS_IA6_RX_PIN 17
#define FS_IA6_SERIAL Serial4
#define FS_IA6_SERIAL_BUFFER_SIZE 256

#define STEPPER_MOTOR_PULSE_PIN 33
#define STEPPER_MOTOR_DIR_PIN 31
#define STEPPER_MOTOR_EN_PIN 32

#define BOMB1_LOCK_SERVO_PIN 7
#define BOMB2_LOCK_SERVO_PIN 6

#define BOMB_LOCK_OPEN 1000
#define BOMB_LOCK_CLOSED 2000

struct FeedbackPacket {
  char start = '\0';
  uint8_t latBefore;
  char latAfter[6];
  uint8_t lngBefore;
  char lngAfter[6];
  float airspeed;               // 4 bytes
  float pressure;
  float batt_volt;
  int16_t roll, pitch, yaw;     // 6 bytes
  uint16_t azimuth;             // 2 bytes
  uint8_t packetConfirmationID = 0; // 1 byte
  uint8_t altitude;             // 1 byte
  uint8_t temp;                 // 1 byte
  uint8_t humidity;             // 1 byte
  uint8_t battery_level;        // 1 byte
  uint8_t m1_throttle;          // 1 byte
  uint8_t m2_throttle;          // 1 byte
  uint8_t m3_throttle;          // 1 byte
  uint8_t m4_throttle;          // 1 byte
} __attribute__((packed));

void doubleToDecimals(double dbl, char floats[6]) {
    
    std::ostringstream strs;
    strs << std::fixed << std::setprecision(6) << dbl;
    std::string out_ = strs.str();
    out_ = out_.substr(out_.find(".") + 1);
    strncpy(floats, out_.c_str(), 6);
    
}

CircularBuffer<uint8_t, FS_IA6_SERIAL_BUFFER_SIZE> radioBuffer;

int sensorsTurn = 0;

TinyGPSPlus gps;
double lat, lng;
TinyGPSHDOP gpsPrecision;

PWMServo bomb1Lock;
PWMServo bomb2Lock;

MechaQMC5883 qmc;

bfs::Ms4525do airSpeedSensor;
const float avg_diff_pres_offset = 128; // in pascals

HTU21D htu;

double temperature, humidity, delta_pressure, delta_pressure_old = 0, air_density, air_speed;

#define IBUS_BUFFSIZE 32    
#define IBUS_MAXCHANNELS 6

uint8_t* buff = (uint8_t*)malloc(28);

static uint8_t ibusIndex = 0;
static uint8_t ibus[IBUS_BUFFSIZE] = {0};
static uint16_t radioValues[IBUS_MAXCHANNELS];
IntervalTimer radioControllerTimer;
void radioControllerRead();
void processRadioController();

IntervalTimer transitionTimer;
void transitionSwitch();
void startTransitioning();
void stopTransitioning();

struct FeedbackPacket packet;

IntervalTimer loraTimer;
void processLora();

File sensorsFile;

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

  rightFirstM.attach(RIGHT_FIRST_MOTOR_PIN, 1000, 2000);
  rightLastM.attach(RIGHT_LAST_MOTOR_PIN, 1000, 2000);
  leftFirstM.attach(LEFT_FIRST_MOTOR_PIN, 1000, 2000);
  leftLastM.attach(LEFT_LAST_MOTOR_PIN, 1000, 2000);

  rightFirstM.write(0);
  rightLastM.write(0);
  leftFirstM.write(0);
  leftLastM.write(0);

  rightWing.attach(RIGHT_WING_SERVO_PIN, 1000, 2000);
  rightWing.write(DEFAULT_SERVO_POS);

  leftWing.attach(LEFT_WING_SERVO_PIN, 1000, 2000);
  leftWing.write(DEFAULT_SERVO_POS);

  rightElevator.attach(RIGHT_ELEVATOR_SERVO_PIN, 1000, 2000);
  rightElevator.write(DEFAULT_SERVO_POS);

  leftElevator.attach(LEFT_ELEVATOR_SERVO_PIN, 1000, 2000);
  leftElevator.write(DEFAULT_SERVO_POS);

  bomb1Lock.attach(BOMB1_LOCK_SERVO_PIN, 1000, 2000);
  bomb1Lock.write(BOMB_LOCK_OPEN);

  bomb2Lock.attach(BOMB2_LOCK_SERVO_PIN, 1000, 2000);
  bomb2Lock.write(BOMB_LOCK_OPEN);

  pinMode(BATTERY_VOLTAGE_PIN, INPUT);

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
  htu.setResolution(HTU21DResolution::RESOLUTION_RH11_T11);
  Serial.println("Initialized HTU21D!");

  if (!bmp.begin()) {
    Serial.println("Couldn't find BMP180!");
    displayError("Couldn't find BMP180!");
    while (1);
  }
  Serial.println("Initialized BMP180!");

  calculateAzimuth();
  firstAzimuth = azimuth;

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("The SD card is not inserted or burned!");
    displayError("Unable to read SD card!");
    while (1) ;
  }

  transitionTimer.priority(0); // has the highest priority, as it is the most important
  pinMode(STEPPER_MOTOR_PULSE_PIN, OUTPUT);
  pinMode(STEPPER_MOTOR_EN_PIN, OUTPUT);
  pinMode(STEPPER_MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(STEPPER_MOTOR_EN_PIN, LOW); // by default the motor is off

  radioControllerTimer.priority(1);
  if (!radioControllerTimer.begin(radioControllerRead, 10000)) {
    Serial.println("Unable to set up a timer for radio controller interrupt!");
    displayError("Unable to set up a timer for radio controller interrupt!");
    while (1);
  }

  setup_lora();

  loraTimer.priority(2);
  if (!loraTimer.begin(processLora, 500000)) { // 1320_000 micros should be enough...
    Serial.println("Unable to set up a timer for lora interrupt!");
    displayError("Unable to set up a timer for lora interrupt!");
    while (1);
  }
  lastLoraPacket = millis();

  displayBigMessage("..Waiting GPS clock..");
  while (!gps.time.isValid() || !gps.time.isUpdated()) {
    while (GPS_SERIAL.available() > 0) {
    gps.encode(GPS_SERIAL.read());
    if (gps.location.isUpdated()) {
      lastGPSPacket = millis();
    }
  }
  }
  std::ostringstream filepath;
  filepath << "sensors " << static_cast<int>(gps.date.year()) << "-" << static_cast<int>(gps.date.month()) << "-" <<
   static_cast<int>(gps.date.day()) << " " << static_cast<int>(gps.time.hour() + 3) << "-" << static_cast<int>(gps.time.minute())
    << "-" << static_cast<int>(gps.time.second()) << ".txt";
  sensorsFile = SD.open(filepath.str().c_str(), FILE_WRITE);
  Serial.println(filepath.str().c_str());
  if (!sensorsFile) {
    Serial.println("Couldn't open the sensors file for writing!");
    displayError("Couldn't open the sensors file!");
    while (1) ;
  }
  Serial.println("Initialized the SD card!");

  displayBigMessage("Finished Initializing");
  delay(2000);

  FS_IA6_SERIAL.clear();
  radioBuffer.clear();
  lastRadioPacket = millis();
}

void loop() {
  auto start = millis();
  
  double delat_pressure_sum = 0;
  double last_diff_pres = 0;

  if (uav_mode != UAV_MODES::failsafe) {
    switch (sensorsTurn) {
      case 0: // GPS
        if (gps.location.isUpdated()) {
          lat = gps.location.lat();
          lng = gps.location.lng();
          Serial.print("Latitude: ");
          Serial.println(lat, 6);
          Serial.print("Longitude: ");
          Serial.println(lng, 6);
          if (gps.hdop.isValid()) {
            //Serial.printf("hdop: %.2f\n", gps.hdop.hdop());
            gpsPrecision = gps.hdop;
          }
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
      air_speed = pow((2 * abs(delta_pressure)) / air_density, 0.5);
      Serial.printf("%.2f delta pres, Humidity: %.2f, Temperature: %.2f, Pressure: %.2f, Air Density: %.3f kg/m^3, air speed: %.2f m/s, altitude: %.2f\n",
       delta_pressure, humidity, temperature, pressure_mb * MB_TO_PA, air_density, air_speed, altitude);
      break;
    case 4: // Display update & azimuth calculation
      calculateAzimuth();
      if (!isFlying()) displaySensorData(temperature, humidity, pressure_mb * MB_TO_PA, azimuth, lat, lng, gps.date, gps.time);
      break;
    case 5: // flush all the files to SD card!
      batt_voltage = (analogRead(BATTERY_VOLTAGE_PIN) / 1023.0 * 3.3 - 0.07) / 3.3 * 25.2; // 0 - 3.23v -> 0 - 25.2v | R1 = 68K, R2 = 10K
      sensorsFile.printf("%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%lf,%lf\n", millis(), temperature, pressure_mb * MB_TO_PA,
       air_density, humidity, altitude, air_speed, azimuth, batt_voltage, lat, lng);
      sensorsFile.flush();
      break;
    case 6: // Lora communication:
      ;
      break;
    }
    sensorsTurn++;
    if (sensorsTurn == 7) {
      sensorsTurn = 0;
    }
  }

  if (isFlying() && isBatteryVoltageLow()) {
    switchToFailSafe();
  }

  if (uav_mode == UAV_MODES::idle) { // in idle mode, we use the first 4 channels to test the controller and the surface control!
    rightWing.write(DEFAULT_SERVO_POS + map((double)radioValues[0], 1000, 2000, -1.0, 1.0) * MAX_SURFACE_CONTROL_ANGLE);
    leftWing.write(DEFAULT_SERVO_POS +  map((double)radioValues[1], 1000, 2000, -1.0, 1.0) * MAX_SURFACE_CONTROL_ANGLE);
    rightElevator.write(DEFAULT_SERVO_POS + map((double)radioValues[2], 1000, 2000, -1.0, 1.0) * MAX_SURFACE_CONTROL_ANGLE);
    leftElevator.write(DEFAULT_SERVO_POS + map((double)radioValues[3], 1000, 2000, -1.0, 1.0) * MAX_SURFACE_CONTROL_ANGLE);
  } else if (uav_mode == UAV_MODES::vtol) {
    ;
  } else if (uav_mode == UAV_MODES::fixed_wing) {
    ;
  } else if (uav_mode == UAV_MODES::failsafe) {
    ;
  }

  calculateRollPitch();
  processRadioController();
  if (!isRadioRunning(radioValues) || !isLoraRunning() || !isGPSRunning()) {
    if (uav_mode == UAV_MODES::failsafe) {
      updateFailSafeMessage();
    } else {
      switchToFailSafe();
    }
  } else {
    exitFailSafeMode();
  }

  Serial.print(millis() - start);
  Serial.println(" ms");

  delay(100);
}

void radioControllerRead() {
  while (FS_IA6_SERIAL.available()) {
    radioBuffer.push(FS_IA6_SERIAL.read());
  }

  //Serial.printf("Recieved %d bytes from GPS\n", GPS_SERIAL.available());
  while (GPS_SERIAL.available() > 0) {
    gps.encode(GPS_SERIAL.read());
    if (gps.location.isUpdated()) {
      lastGPSPacket = millis();
    }
  }
}

// called when needed to move the stepper motor to start moving to the other mode
void startTransitioning() {
  transitionTimer.begin(transitionSwitch, 1000);
  digitalWrite(STEPPER_MOTOR_EN_PIN, HIGH);
}

void stopTransitioning() {
  transitionTimer.end();
  digitalWrite(STEPPER_MOTOR_EN_PIN, LOW);
}

void transitionSwitch() {

  static bool transitionPulseState = false;
  digitalWrite(STEPPER_MOTOR_PULSE_PIN, transitionPulseState);
  transitionPulseState = !transitionPulseState;

}

void processLora() {
  packet.airspeed = (float)air_speed;
  packet.altitude = (uint8_t)altitude;
  packet.azimuth = azimuth;
  packet.pressure = pressure_mb * MB_TO_PA / 1000.0;
  packet.battery_level = 100;
  packet.humidity = humidity;
  packet.latBefore = (uint8_t)lat;
  packet.lngBefore = (uint8_t)lng;
  doubleToDecimals(lat, packet.latAfter);
  doubleToDecimals(lng, packet.lngAfter);
  packet.m1_throttle = 0;
  packet.m2_throttle = 0;
  packet.m3_throttle = 0;
  packet.m4_throttle = 0;
  packet.batt_volt = batt_voltage;
  packet.roll = kalm_roll;
  packet.pitch = kalm_pitch;
  packet.yaw = yaw;
  packet.temp = temperature;

  E32_transmitter.sendFixedMessage(CONTROL_STATION_ADDH, CONTROL_STATION_ADDL, CONTROL_STATION_CHANNEL, (uint8_t*)&packet, sizeof(packet));
  if (E32_receiver.available() >= 28) {
    Serial.print("Received packet: ");
    LORA_RECEIVER_SERIAL.readBytes(buff, 27);
    Serial.println((char*)buff);
    lastLoraPacket = millis();
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
}
