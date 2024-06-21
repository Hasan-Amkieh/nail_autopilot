#include "SFE_BMP180.h"

SFE_BMP180 bmp;
double T, pressure_mb, altitude0 = 0, altitude;
char bmpStatus;

void calculatePressureAndAltitude() {

  bmpStatus = bmp.startTemperature();
  if (bmpStatus != 0) {
    delay(bmpStatus);
    bmpStatus = bmp.getTemperature(T);
    if (bmpStatus != 0) {
      Serial.print("Temperature :");
      Serial.print(T, 2);
      Serial.println("*c");
    }
    bmpStatus = bmp.startPressure(3);// 0 to 3
    if (bmpStatus != 0) {
      delay(bmpStatus);
      bmpStatus = bmp.getPressure(pressure_mb, T);
      if (bmpStatus != 0) {
        Serial.print("absolute pressure: ");
        Serial.print(pressure_mb * 100.0, 2);
        Serial.println("pa");
      }
 
      altitude = bmp.altitude(pressure_mb, 500);
      if (altitude0 == 0) {
        altitude0 = altitude;
      }
      altitude = abs(altitude0 - altitude);
    }
  }

}
