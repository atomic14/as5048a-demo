#include <Arduino.h>
#include <U8g2lib.h>
#include "AS5048A.h"

AS5048A angleSensor(23, 22, 21, 19);
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R2, 15, 4, 16);

void setup()
{
  Serial.begin(115200);
  angleSensor.begin();
  u8g2.begin();
  u8g2.setFont(u8g2_font_logisoso24_tf);
  u8g2.setBusClock(10000000);
}

int infoCount = 0;

void loop()
{
  delay(100);
  double rotation = angleSensor.getRotationInDegrees();
  char text[100];
  sprintf(text, "%.2f", rotation);
  u8g2.clearBuffer();
  u8g2.drawUTF8(0, 64 - 20, text);
  u8g2.sendBuffer();
}