#include <Adafruit_SSD1306.h>

#define OLED_RESET 9
#define OLED_SA0   8

Adafruit_SSD1306 display(OLED_RESET, OLED_SA0);

void setup()
{
  delay(1000);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done
  
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the spxdd d lashscreen.
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(15,0);
  display.println("Alpha v2");
  display.setCursor(10,25);
  display.println("Test");
  display.setTextSize(1);
  display.setCursor(10,55);
  display.println("Press to calibrate");
  display.display();
  
}

void loop()
{
}
