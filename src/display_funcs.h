#include <U8g2lib.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
void printTextOverflow(const char *msg, int xloc, int yloc);

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

void displayBigMessage(String toPrint) {

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  u8g2.setCursor(0, 22);
  u8g2.print(toPrint);

  u8g2.sendBuffer();

}

void displayError(String error) {

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  u8g2.setCursor(0, 12);
  u8g2.print("ERROR: ");

  u8g2.setFont(u8g2_font_profont10_mr);

  printTextOverflow(error.c_str(), 0, 24);
  u8g2.sendBuffer();

}

// display a string on multiple lines, keeping words intact where possible
void printTextOverflow(const char *msg, int xloc, int yloc) {

   int dspwidth = u8g2.getDisplayWidth();                        // display width in pixels
   int strwidth = 0;                         // string width in pixels
   char glyph[2]; glyph[1] = 0;

   for (const char *ptr = msg, *lastblank = NULL; *ptr; ++ptr) {
      while (xloc == 0 && *msg == ' ')
         if (ptr == msg++) ++ptr;                        // skip blanks at the left edge

      glyph[0] = *ptr;
      strwidth += u8g2.getStrWidth(glyph);                        // accumulate the pixel width
      if (*ptr == ' ')  lastblank = ptr;                        // remember where the last blank was
      else ++strwidth;                        // non-blanks will be separated by one additional pixel

      if (xloc + strwidth > dspwidth) {                        // if we ran past the right edge of the display
         int starting_xloc = xloc;
         while (msg < (lastblank ? lastblank : ptr)) {                        // print to the last blank, or a full line
            glyph[0] = *msg++;
            xloc += u8g2.drawStr(xloc, yloc, glyph); 
         }

         strwidth -= xloc - starting_xloc;                        // account for what we printed
         yloc += u8g2.getMaxCharHeight();                        // advance to the next line
         xloc = 0; lastblank = NULL;
      }
   }
   while (*msg) {                        // print any characters left over
      glyph[0] = *msg++;
      xloc += u8g2.drawStr(xloc, yloc, glyph);
   }
}
