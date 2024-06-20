#include <Arduino.h>
#include "ms4525do.h"
#include <Adafruit_BMP280.h>
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

#define ANKARA_PRESSURE 938 // meters from the sea in Ankara

// TODO: Delete them as I am applying the new system:
#define HUMIDITY_PRESSURE_INTERVAL 500
#define AIRSPEED_INTERVAL 500
#define MAG_INTERVAL 500
#define INFO_TO_CONTROL_STATION_INTERVAL 600
#define DISPLAY_INTERVAL 1000

#define GPS_TX_PIN 28
#define GPS_RX_PIN 29
#define GPS_SERIAL Serial7

#define FS_IA6_TX_PIN 34
#define FS_IA6_RX_PIN 35
#define FS_IA6 Serial8

TinyGPSPlus gps;

MechaQMC5883 qmc;

bfs::Ms4525do airSpeedSensor;
const float avg_diff_pres_offset = 128; // in pascals

HTU21D htu;
Adafruit_BMP280 bmp;

double temperature, pressure, old_pressure = 0, humidity, delta_pressure, delta_pressure_old = 0, air_density;

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

  bmp = Adafruit_BMP280(&Wire);
  if (!bmp.begin(0x76, 0x58)) {
    Serial.println("Couldn't find BMP180!");
    displayError("Couldn't find BMP180!");
    while (1);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1);   /* Standby time. */
  Serial.println("Initialized BMP180!");

  displayBigMessage("Finished Initializing");
  delay(1000);
}

void loop() {
  auto start = millis();
  
  if (true) {
    Serial.printf("Received %d bytes from GPS\n", GPS_SERIAL.available());
    while (GPS_SERIAL.available() > 0) {
      gps.encode(GPS_SERIAL.read());
    }

    if (gps.location.isUpdated()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
  }

    double sum = 0;
    const int samples = 10;
    double last = 0;
    for (int i = 0 ; i < samples ; i++) {
      if (airSpeedSensor.Read()) {
        last = airSpeedSensor.pres_pa();
      }
      sum += last;
    }
    delta_pressure = low_pass_filter(delta_pressure_old, sum / samples + avg_diff_pres_offset);
    delta_pressure_old = delta_pressure;
    if (delta_pressure > 0) {
      delta_pressure = 0;
    }

    if (!htu.measure()) {
      Serial.println("WARNING: Unable to measure the humidity!");
    }
    humidity = htu.getHumidity();
    pressure = low_pass_filter(old_pressure, bmp.readPressure());
    old_pressure = pressure;
    temperature = bmp.readTemperature();
    air_density = densityhumidair(pressure, temperature, humidity / 100);
    Serial.printf("%.2f delta pres, Humidity: %.2f, Temperature: %.2f, Pressure: %.2f, Air Density: %.3f kg/m^3, air speed: %.2f m/s, altitude: %.2f\n",
     delta_pressure, humidity, temperature, pressure, air_density, pow((2 * abs(delta_pressure)) / air_density, 0.5), bmp.readAltitude(ANKARA_PRESSURE));
    displaySensorData(temperature, humidity, pressure, azimuth, true, gps.location.lat(), gps.location.lng());
  }

  print_roll_pitch();
  Serial.print(millis() - start);
  Serial.println(" ms");

  delay(100);
}
