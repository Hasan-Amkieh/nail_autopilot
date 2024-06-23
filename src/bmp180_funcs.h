#include "SFE_BMP180.h"

SFE_BMP180 bmp;
double T, pressure_mb, altitude0 = 0, altitude;
char bmpStatus;

void calculatePressureAndAltitude() {

  bmpStatus = bmp.startTemperature();
  if (bmpStatus != 0) {
    delay(bmpStatus);
    bmpStatus = bmp.getTemperature(T);
    bmpStatus = bmp.startPressure(3);// 0 to 3
    if (bmpStatus != 0) {
      delay(bmpStatus);
      bmpStatus = bmp.getPressure(pressure_mb, T);
      altitude = bmp.altitude(pressure_mb, 500);
      if (altitude0 == 0) {
        altitude0 = altitude;
      }
      altitude = abs(altitude0 - altitude);
    }
  }

}
