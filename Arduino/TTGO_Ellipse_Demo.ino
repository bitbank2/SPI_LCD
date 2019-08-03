#include <spi_lcd.h>
void setup() {
   spilcdInit(LCD_ST7789_135, 0, 0, 40000000, 5, 16, -1, 4, -1, 19, 18); // TTGO T-Display pin numbering, 40Mhz
} // setup

void loop() {
int x, y, r1, r2;
uint16_t usColor;

#define WIDTH 135
#define HEIGHT 240

  while (1)
  {
    uint16_t usColor = random(65536);
    x = random(WIDTH);
    y = random(HEIGHT);
    r1 = random(4, WIDTH/2);
    r2 = random(4, WIDTH/2);
    spilcdEllipse(x, y, r1, r2, usColor,1);
  }
} // loop
