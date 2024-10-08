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

#define VTOL_TRANSITION LOW
#define FW_TRANSITION HIGH
#define STEPPER_MOTOR_PULSE_PIN 33
#define STEPPER_MOTOR_DIR_PIN 31 // LOW to VTOL, HIGH to FW
//#define STEPPER_MOTOR_EN_PIN 32 // unused, can be used for other stuff
#define STEPPER_MOTOR_INTERVAL 200 // microseconds
#define TO_VTOL_DIR LOW
#define TO_FW_DIR HIGH
#define FULL_TRANSITION_STEPS_COUNT 13500 // at 400 steps per revolution with DM556

#define BOMB1_LOCK_SERVO_PIN 7
#define BOMB2_LOCK_SERVO_PIN 6

#define BOMB_LOCK_OPEN 0.0
#define BOMB_LOCK_CLOSED 1.0

struct FeedbackPacket {
  char start = 'A';
  uint8_t uav_mode = 0; // 0 - idle, 1 - vtol, 2 - fw, 3 - transitioning, 4 - failsafe
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

SignalGen bomb1Lock(BOMB1_LOCK_SERVO_PIN);
SignalGen bomb2Lock(BOMB2_LOCK_SERVO_PIN);

MechaQMC5883 qmc;

bfs::Ms4525do airSpeedSensor;
float avg_diff_pres_offset = 0; // in pascals

HTU21D htu;

double temperature, humidity, delta_pressure = 0, delta_pressure_old = 0, air_density, air_speed;

uint8_t* buff = (uint8_t*)malloc(3);

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

  rightFirstM.begin(50);
  rightLastM.begin(50);
  leftFirstM.begin(50);
  leftLastM.begin(50);

  rightFirstM.setDutyCycle(0.0);
  rightLastM.setDutyCycle(0.0);
  leftFirstM.setDutyCycle(0.0);
  leftLastM.setDutyCycle(0.0);

  rightWingAil.begin(50);
  rightWingAil.setDutyCycle(DEFAULT_SERVO_POS_RA);

  leftWingAil.begin(50);
  leftWingAil.setDutyCycle(DEFAULT_SERVO_POS_LA);

  rightElevator.begin(50);
  rightElevator.setDutyCycle(DEFAULT_SERVO_POS_RE);

  leftElevator.begin(50);
  leftElevator.setDutyCycle(DEFAULT_SERVO_POS_LE);

  bomb1Lock.begin(50);
  bomb1Lock.setDutyCycle(BOMB_LOCK_OPEN);

  bomb2Lock.begin(50);
  bomb2Lock.setDutyCycle(BOMB_LOCK_OPEN);

  pinMode(BATTERY_VOLTAGE_PIN, INPUT);

  testAllServos();

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

  /*airSpeedSensor.Config(&Wire, 0x28, 1.0f, -1.0f);
  if (!airSpeedSensor.Begin()) {
    Serial.println("Couldn't find Air speed!");
    displayError("Couldn't find Air speed!");
    while(1){}
  }
  float last_diff_pres = 0, delat_pressure_sum = 0;
  for (int i = 0 ; i < 100 ; i++) {
    if (airSpeedSensor.Read()) {
      last_diff_pres = airSpeedSensor.pres_pa();
    }
    delat_pressure_sum += last_diff_pres;
  }
  avg_diff_pres_offset = delat_pressure_sum / 100.0;
  Serial.printf("Initialized Airspeed sensor!delta pressure offset: %2f\n", avg_diff_pres_offset);*/

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
  pinMode(STEPPER_MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(STEPPER_MOTOR_PULSE_PIN, LOW);

  /*displayBigMessage("Running steps");
  while (true) {
    digitalWrite(STEPPER_MOTOR_DIR_PIN, HIGH);
    startTransitioning();
    delay(500);
    stopTransitioning();
  }*/

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

uint32_t start;

void loop() {
  start = micros();
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time) / 1000000.0;
  
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
      /*for (int i = 0 ; i < 10 ; i++) {
        if (airSpeedSensor.Read()) {
          last_diff_pres = airSpeedSensor.pres_pa();
        }
        delat_pressure_sum += last_diff_pres;
      }
      delta_pressure = low_pass_filter(delta_pressure_old, delat_pressure_sum / 10 - avg_diff_pres_offset);
      delta_pressure_old = delta_pressure;*/
      if (delta_pressure > 0) {
        delta_pressure = 0;
      }
      //air_speed = pow((2 * abs(delta_pressure)) / air_density, 0.5);
      air_speed = 0;
      Serial.printf("%.2f delta pres, Humidity: %.2f, Temperature: %.2f, Pressure: %.2f, Air Density: %.3f kg/m^3, air speed: %.2f m/s, altitude: %.2f\n",
       delta_pressure, humidity, temperature, pressure_mb * MB_TO_PA, air_density, air_speed, altitude);
      break;
    case 4: // Display update & azimuth calculation
      calculateAzimuth();
      if (!isFlying()) displaySensorData(temperature, humidity, pressure_mb * MB_TO_PA, azimuth, lat, lng, gps.date, gps.time);
      break;
    case 5: // flush all the files to SD card!
      batt_voltage = (analogRead(BATTERY_VOLTAGE_PIN) / 1023.0 * 3.3 - 0.07) / 3.3 * 25.2 + 1; // 0 - 3.23v -> 0 - 25.2v | R1 = 68K, R2 = 10K
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

  calculateRollPitch();

  if (uav_mode == UAV_MODES::idle) { // in idle mode, we use the 6 channels to test the controller and the surface control!
    surface_controls[0] = DEFAULT_SERVO_POS_RA + map((double)radioValues[0], 1000, 2000, -1.0, 1.0) * MAX_AILERON_DUTY_R;
    surface_controls[1] = DEFAULT_SERVO_POS_LA + map((double)radioValues[0], 1000, 2000, -1.0, 1.0) * MAX_AILERON_DUTY_L;
    surface_controls[2] = DEFAULT_SERVO_POS_RE + map((double)radioValues[1], 1000, 2000, -1.0, 1.0) * MAX_ELEVATOR_DUTY_R;
    surface_controls[3] = DEFAULT_SERVO_POS_LE + map((double)radioValues[1], 1000, 2000, -1.0, 1.0) * MAX_ELEVATOR_DUTY_L;
    commandSurfaceControls();

    motor_throttles[0] = map(radioValues[2], 1000, 2000, 0.0, 1.0);
    if (motor_throttles[0] > 0.2) {
      motor_throttles[0] = 0.2;
    }
    motor_throttles[1] = motor_throttles[0];
    motor_throttles[2] = motor_throttles[0];
    motor_throttles[3] = motor_throttles[0];
    commandMotors();

    targetSteps = map(radioValues[4], 1000, 2000, 0, FULL_TRANSITION_STEPS_COUNT);
    if (targetSteps != stepsCounter) {
      startTransitioning();
    }

    if (radioValues[5] == 1500) {
      bomb1Lock.setDutyCycle(BOMB_LOCK_CLOSED);
      bomb2Lock.setDutyCycle(BOMB_LOCK_OPEN);
    } else if (radioValues[5] == 2000) {
      bomb1Lock.setDutyCycle(BOMB_LOCK_CLOSED);
      bomb2Lock.setDutyCycle(BOMB_LOCK_CLOSED);
    } else {
      bomb1Lock.setDutyCycle(BOMB_LOCK_OPEN);
      bomb2Lock.setDutyCycle(BOMB_LOCK_OPEN);
    }
  } else if (uav_mode == UAV_MODES::vtol) {
    getDesStateVTOL();
    controlAngleVTOL();
    controlMixerVTOL();
    commandMotors();
  } else if (uav_mode == UAV_MODES::fixed_wing) {
    getDesStateFW();
    controlAngleFW();
    controlMixerFW();
    commandMotors();
  } else if (uav_mode == UAV_MODES::transitioning) {
    // TODO:
    ;
  } else if (uav_mode == UAV_MODES::failsafe) {
    if (prevMode == UAV_MODES::idle) {
      motor_throttles[0] = 0;
      motor_throttles[1] = 0;
      motor_throttles[2] = 0;
      motor_throttles[3] = 0;
      commandMotors();
    } else if (prevMode == UAV_MODES::vtol) {
      // TODO:
      motor_throttles[0] = 0;
      motor_throttles[1] = 0;
      motor_throttles[2] = 0;
      motor_throttles[3] = 0;
      commandMotors();
    } else if (prevMode == UAV_MODES::fixed_wing) {
      motor_throttles[0] = 0;
      motor_throttles[1] = 0;
      motor_throttles[2] = 0;
      motor_throttles[3] = 0;
      surface_controls[0] = MAX_AILERON_DUTY_R; // The ailerons has to be rolling to the right side
      surface_controls[1] = -MAX_AILERON_DUTY_L;
      surface_controls[2] = -MAX_ELEVATOR_DUTY_R; // The elevators has to gain altitude, acts as air brakes
      surface_controls[3] = -MAX_ELEVATOR_DUTY_L;
      commandMotors();
      commandSurfaceControls();
    }
  }

  processRadioController();
  if (idleModeRadioLock && radioValues[0] >= 1450 && radioValues[0] <= 1550 && radioValues[1] >= 1450 && radioValues[1] <= 1550
  && radioValues[3] >= 1450 && radioValues[3] <= 1550 && radioValues[2] <= 1020 && radioValues[4] == 1000 && radioValues[5] == 1000) {
    idleModeRadioLock = false;
  }
  if (vtolModeRadioLock && radioValues[0] >= 1450 && radioValues[0] <= 1550 && radioValues[1] >= 1450 && radioValues[1] <= 1550
  && radioValues[3] >= 1450 && radioValues[3] <= 1550 && radioValues[2] <= 1020 && radioValues[4] == 1000 && radioValues[5] == 1000) {
    vtolModeRadioLock = false;
  }

  if (!isRadioRunning(radioValues) || !isLoraRunning() || !isGPSRunning() || isRadioLock()) {
    if (uav_mode == UAV_MODES::failsafe) {
      updateFailSafeMessage();
    } else {
      switchToFailSafe();
    }
  } else {
    exitFailSafeMode();
  }

  Serial.print((micros() - start) / 1000);
  Serial.println(" ms");

  //delay(100);
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
  if (targetSteps < stepsCounter) {
    digitalWrite(STEPPER_MOTOR_DIR_PIN, TO_VTOL_DIR);
  } else {
    digitalWrite(STEPPER_MOTOR_DIR_PIN, TO_FW_DIR);
  }
  transitionTimer.begin(transitionSwitch, STEPPER_MOTOR_INTERVAL);
}

void stopTransitioning() {
  transitionTimer.end();
}

void transitionSwitch() {

  static bool transitionPulseState = false;
  digitalWrite(STEPPER_MOTOR_PULSE_PIN, transitionPulseState);
  transitionPulseState = !transitionPulseState;
  stepsCounter++;
  if (stepsCounter == targetSteps) {
    stopTransitioning();
  }

}

void processLora() {
  packet.uav_mode = uav_mode;
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
  packet.m1_throttle = (uint8_t)map(motor_throttles[0], 0.0, 1.0, 0, 100);
  packet.m2_throttle = (uint8_t)map(motor_throttles[1], 0.0, 1.0, 0, 100);
  packet.m3_throttle = (uint8_t)map(motor_throttles[2], 0.0, 1.0, 0, 100);
  packet.m4_throttle = (uint8_t)map(motor_throttles[3], 0.0, 1.0, 0, 100);
  packet.batt_volt = batt_voltage;
  packet.roll = kalm_roll;
  packet.pitch = kalm_pitch;
  packet.yaw = yaw;
  packet.temp = temperature;

  E32_transmitter.sendFixedMessage(CONTROL_STATION_ADDH, CONTROL_STATION_ADDL, CONTROL_STATION_CHANNEL, (uint8_t*)&packet, sizeof(packet));
  if (LORA_RECEIVER_SERIAL.available() >= 1) {
    Serial.print("Received packet: ");
    LORA_RECEIVER_SERIAL.readBytes(buff, 1);
    Serial.println(String((char*)buff));
    lastLoraPacket = millis();
    if (buff[0] == '1' && uav_mode == UAV_MODES::idle) { // autonomous mission command
      uav_mode = UAV_MODES::vtol;
      isRadioLockedVTOL = true;
      vtolModeRadioLock = true;
      Serial.println("Switching to VTOL in manual mode!");
    }
    if (buff[0] == '2' && uav_mode == UAV_MODES::idle) { // manual mission command
      uav_mode = UAV_MODES::vtol;
      isRadioLockedVTOL = true;
      vtolModeRadioLock = true;
      Serial.println("Switching to VTOL in manual mode!");
    }
    if (buff[0] == '3' && isFlying()) {
      bomb1Lock.setDutyCycle(BOMB_LOCK_CLOSED);
      Serial.println("Dropping first bomb!");
    }
    if (buff[0] == '4' && isFlying()) {
      bomb2Lock.setDutyCycle(BOMB_LOCK_CLOSED);
      Serial.println("Dropping second bomb!");
    }
    if (buff[0] == '5' && isFlying()) {
      Serial.println("switching to manual mode!");
    }
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
