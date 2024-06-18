#include <U8g2lib.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void displaySensorData(float temp, float hum, float pressure, float azimuth, bool isGPSConnected, bool isControllerArmed) {
  u8g2.clearBuffer(); // Clear the internal memory of the display
  
  u8g2.setFont(u8g2_font_profont10_mr);
  
  u8g2.setCursor(0, 8);
  u8g2.print("Temp: ");
  u8g2.print(temp, 2);
  u8g2.println(" C");
  
  u8g2.setCursor(0, 16);
  u8g2.print("Humidity: ");
  u8g2.print(hum, 2);
  u8g2.println(" %");
  
  u8g2.setCursor(0, 24);
  u8g2.print("Pressure: ");
  u8g2.print(pressure / 1000.0, 3);
  u8g2.println(" kPa");
  
  u8g2.setCursor(0, 32);
  u8g2.print("GPS: ");
  u8g2.println(isGPSConnected ? "Connected" : "Disconnected");
  
  u8g2.setCursor(0, 40);
  u8g2.print("Controller: ");
  u8g2.println(isControllerArmed ? "Armed" : "Not Armed");
  
  u8g2.setCursor(0, 50);
  u8g2.print("Azimuth: ");
  u8g2.print(azimuth, 2);
  u8g2.setFont(u8g_font_courB10);
  u8g2.print("\xb0");
  
  u8g2.sendBuffer(); // Transfer internal memory to the display
}