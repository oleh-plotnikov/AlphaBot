#include "nimble_display.h"
#include <Adafruit_SSD1306.h>

void testdrawchar(const char c[]);

Adafruit_SSD1306 display(OLED_RESET, OLED_SA0);

void nimble_display_init()
{
  Serial.begin(115200);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(WHITE);
  display.setCursor(15,20);
  display.println("Nimble");
  display.display();
  delay(3000);
  display.clearDisplay();
}

void nimble_debug_print(const char c[], int type)
{
  if(type == eNIMBLE_SERIAL)
  {
    Serial.println(c);
  }
  else if (type == eNIMBLE_LCD)
  {
    display.clearDisplay();
    testdrawchar(c);
  }
}

void testdrawchar(const char c[]) {
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print(c);
  display.display();
}
