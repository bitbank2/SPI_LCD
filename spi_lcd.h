#ifndef SPI_LCD_H
#define SPI_LCD_H
//
// SPI_LCD using the SPI interface
// Copyright (c) 2017 Larry Bank
// email: bitbank@pobox.com
// Project started 4/25/2017
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// Initialize the library
int spilcdInit(int iLCDType, int iSPIChannel, int iDCPin, int iResetPin, int iLEDPin);

// Turns off the display and frees the resources
void spilcdShutdown(void);

// Fills the display with the byte pattern
int spilcdFill(unsigned short usPattern);

//
// Reset the scroll position to 0
//
void spilcdScrollReset(void);

// Configure a GPIO pin for input
// Returns 0 if successful, -1 if unavailable
int spilcdConfigurePin(int iPin);

// Read from a GPIO pin
int spilcdReadPin(int iPin);

//
// Scroll the screen N lines vertically (positive or negative)
// This is a delta which affects the current hardware scroll offset
// If iFillcolor != -1, the newly exposed lines will be filled with that color
//
void spilcdScroll(int iLines, int iFillColor);

// Draw a 16x16 tile
int spilcdDrawTile(int x, int y, unsigned char *pTile, int iPitch);

// Draw a 16x16 tile scaled to 24x24 with pixel averaging
int spilcdDrawScaledTile(int x, int y, unsigned char *pTile, int iPitch);

// Write a text string to the display at x (column 0-83) and y (row 0-5)
// bLarge = 0 - 8x8 font, bLarge = 1 - 16x24 font
int spilcdWriteString(int x, int y, char *szText, unsigned short usFGColor, unsigned short usBGColor, int bLarge);

// Sets a pixel to the given color
// Coordinate system is pixels, not text rows (0-239, 0-319)
int spilcdSetPixel(int x, int y, unsigned short usPixel);

// Set the software orientation
int spilcdSetOrientation(int iOrientation);

//
// Treat the LCD as a 240x320 portrait-mode image
// or a 320x240 landscape mode image
// This affects the coordinate system and rotates the
// drawing direction of fonts and tiles
//
#define LCD_ORIENTATION_PORTRAIT 1
#define LCD_ORIENTATION_LANDSCAPE 2
#define LCD_ILI9341 1
#define LCD_HX8357 2

#endif // SPI_LCD_H
