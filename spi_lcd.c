// LCD direct communication using the SPI interface
// Copyright (c) 2017 Larry Bank
// email: bitbank@pobox.com
// Project started 5/15/2017
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

// The ILITEK LCD display controllers communicate through the SPI interface
// and two GPIO pins to control the RESET, and D/C (data/command)
// control lines. 

// Use one of the following 3 libraries for talking to the SPI/GPIO
#define USE_PIGPIO
//#define USE_BCM2835
//#define USE_WIRINGPI

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#ifdef USE_WIRINGPI
#include <wiringPi.h>
#include <wiringPiSPI.h>
#endif // USE_WIRINGPI
#include <linux/types.h>
//#include <linux/spi/spidev.h>
#ifdef USE_BCM2835
#include <bcm2835.h>
#endif
#include "spi_lcd.h"
#ifdef USE_PIGPIO
#include <pigpio.h>
#endif // USE_PIGPIO

extern unsigned char ucFont[];
static int file_spi = -1; // SPI system handle
static int iDCPin, iResetPin, iLEDPin; // pin numbers for the GPIO control lines
static int iScrollOffset; // current scroll amount
static int iOrientation = LCD_ORIENTATION_PORTRAIT; // default to 'natural' orientation
static int iLCDType;
static int iWidth, iHeight;

static void spilcdWriteCommand(unsigned char);
static void spilcdWriteData8(unsigned char c);
static void spilcdWriteData16(unsigned short us);
static void spilcdSetPosition(int x, int y, int w, int h);
static void spilcdWriteDataBlock(unsigned char *pData, int iLen);
static void myPinWrite(int iPin, int iValue);
int spilcdFill(unsigned short usData);

#ifdef USE_PIGPIO
static unsigned char ucPIGPins[] = {0xff,0xff,0xff,2,0xff,3,0xff,4,14,0xff,15,
                       17,18,27,0xff,22,23,0xff,24,10,0xff,9,25,11,8,0xff,7,0,1,
                       5,0xff,6,12,13,0xff,19,16,26,20,0xff,21};
#endif // USE_PIGPIO

#ifdef USE_WIRINGPI
static unsigned char ucWPPins[] = {0xff,0xff,0xff,8,0xff,9,0xff,7,15,0xff,16,0,1,
        2,0xff,3,4,0xff,5,12,0xff,13,6,14,10,0xff,11,30,31,21,0xff,22,26,23,0xff,24,27,25,28,0xff,29};
#endif // USE_WIRINGPI

#ifdef USE_BCM2835
static unsigned char ucBCM2835Pins[] = {0,0,0,RPI_V2_GPIO_P1_03,0,RPI_V2_GPIO_P1_05,0,
	RPI_V2_GPIO_P1_07, RPI_V2_GPIO_P1_08,0, RPI_V2_GPIO_P1_10, RPI_V2_GPIO_P1_11,
        RPI_V2_GPIO_P1_12, RPI_V2_GPIO_P1_13, 0, RPI_V2_GPIO_P1_15,RPI_V2_GPIO_P1_16,
        0, RPI_V2_GPIO_P1_18, RPI_V2_GPIO_P1_19, 0, RPI_V2_GPIO_P1_21, RPI_V2_GPIO_P1_22,
        RPI_V2_GPIO_P1_23, RPI_V2_GPIO_P1_24, 0, RPI_V2_GPIO_P1_26, 0, 0,
        RPI_V2_GPIO_P1_29, 0, RPI_V2_GPIO_P1_31, RPI_V2_GPIO_P1_32, RPI_V2_GPIO_P1_33,
        0, RPI_V2_GPIO_P1_35, RPI_V2_GPIO_P1_36, RPI_V2_GPIO_P1_37, RPI_V2_GPIO_P1_38,
        0, RPI_V2_GPIO_P1_40};
#endif // BCM2835

//
// Default SPI transmission frequency
// 32Mhz seems to be the maximum that the RPi can handle
// I've seen others claim that the ILI9341 is capable of
// faster speeds, but it doesn't appear to work at 62Mhz
// On the Raspberry Pi
//

typedef enum
{
 MODE_DATA = 0,
 MODE_COMMAND
} DC_MODE;

// Sets the D/C pin to data or command mode
void spilcdSetMode(int iMode)
{
	myPinWrite(iDCPin, iMode == MODE_DATA);
} /* spilcdSetMode() */

// List of command/parameters to initialize the ili9341 display
static unsigned char uc240InitList[] = {
        4, 0xEF, 0x03, 0x80, 0x02,
        4, 0xCF, 0x00, 0XC1, 0X30,
        5, 0xED, 0x64, 0x03, 0X12, 0X81,
        4, 0xE8, 0x85, 0x00, 0x78,
        6, 0xCB, 0x39, 0x2C, 0x00, 0x34, 0x02,
        2, 0xF7, 0x20,
        3, 0xEA, 0x00, 0x00,
        2, 0xc0, 0x23, // Power control
        2, 0xc1, 0x10, // Power control
        3, 0xc5, 0x3e, 0x28, // VCM control
        2, 0xc7, 0x86, // VCM control2
        2, 0x36, 0x48, // Memory Access Control
        2, 0x3a, 0x55,
        3, 0xb1, 0x00, 0x18,
        4, 0xb6, 0x08, 0x82, 0x27, // Display Function Control
        2, 0xF2, 0x00, // Gamma Function Disable
        2, 0x26, 0x01, // Gamma curve selected
        16, 0xe0, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08,
                0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00, // Set Gamma
        16, 0xe1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07,
                0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F, // Set Gamma
        3, 0xb1, 0x00, 0x10, // FrameRate Control 119Hz
        0
};

// List of command/parameters to initialize the st7735 display
static unsigned char uc128InitList[] = {
//	4, 0xb1, 0x01, 0x2c, 0x2d,	// frame rate control
//	4, 0xb2, 0x01, 0x2c, 0x2d,	// frame rate control (idle mode)
//	7, 0xb3, 0x01, 0x2c, 0x2d, 0x01, 0x2c, 0x2d, // frctrl - partial mode
//	2, 0xb4, 0x07,	// non-inverted
//	4, 0xc0, 0x82, 0x02, 0x84,	// power control
//	2, 0xc1, 0xc5, 	// pwr ctrl2
//	2, 0xc2, 0x0a, 0x00, // pwr ctrl3
//	3, 0xc3, 0x8a, 0x2a, // pwr ctrl4
//	3, 0xc4, 0x8a, 0xee, // pwr ctrl5
//	2, 0xc5, 0x0e,		// pwr ctrl
//	1, 0x20,	// display inversion off
	2, 0x3a, 0x55,	// pixel format RGB565
	2, 0x36, 0xc0, // MADCTL
	17, 0xe0, 0x09, 0x16, 0x09,0x20,
		0x21,0x1b,0x13,0x19,
		0x17,0x15,0x1e,0x2b,
		0x04,0x05,0x02,0x0e, // gamma sequence
	17, 0xe1, 0x0b,0x14,0x08,0x1e,
		0x22,0x1d,0x18,0x1e,
		0x1b,0x1a,0x24,0x2b,
		0x06,0x06,0x02,0x0f,
	0
};
// List of command/parameters to initialize the hx8357 display
static unsigned char uc480InitList[] = {
	2, 0x3a, 0x55,
	2, 0xc2, 0x44,
	5, 0xc5, 0x00, 0x00, 0x00, 0x00,
	16, 0xe0, 0x0f, 0x1f, 0x1c, 0x0c, 0x0f, 0x08, 0x48, 0x98, 0x37,
		0x0a,0x13, 0x04, 0x11, 0x0d, 0x00,
	16, 0xe1, 0x0f, 0x32, 0x2e, 0x0b, 0x0d, 0x05, 0x47, 0x75, 0x37,
		0x06, 0x10, 0x03, 0x24, 0x20, 0x00,
	16, 0xe2, 0x0f, 0x32, 0x2e, 0x0b, 0x0d, 0x05, 0x47, 0x75, 0x37,
		0x06, 0x10, 0x03, 0x24, 0x20, 0x00,
	2, 0x36, 0x48,
	0	
};

//
// Wrapper function for writing to SPI
//
static void myspiWrite(unsigned char *pBuf, int iLen)
{
    
#ifdef USE_PIGPIO
    spiWrite(file_spi, (char *)pBuf, iLen);
#endif
#ifdef USE_BCM2835
    bcm2835_spi_writenb((char *)pBuf, iLen);
#endif
#ifdef USE_WIRINGPI
    write(file_spi, (char *)pBuf, iLen);
#endif
    
} /* myspiWrite() */

//
// Wrapper function to control a GPIO line
//
static void myPinWrite(int iPin, int iValue)
{
#ifdef USE_BCM2835
	if (iValue)
		bcm2835_gpio_set(iPin);
	else
		bcm2835_gpio_clr(iPin);
#endif

#ifdef USE_PIGPIO
	gpioWrite(iPin, iValue);
#endif

#ifdef USE_WIRINGPI
	digitalWrite(iPin, (iValue) ? HIGH: LOW);
#endif
} /* myPinWrite() */

//
// Initialize the LCD controller and clear the display
//
int spilcdInit(int iType, int iChannel, int iSPIFreq, int iDC, int iReset, int iLED)
{
unsigned char *s;
int i, iCount;

	if (iType != LCD_ILI9341 && iType != LCD_ST7735 && iType != LCD_HX8357)
	{
		printf("Unsupported display type\n");
		return -1;
	}
	iLCDType = iType;
	iScrollOffset = 0; // current hardware scroll register value

#ifdef USE_BCM2835
     iDCPin = ucBCM2835Pins[iDC]; // use the pin numbers as-is
     iResetPin = ucBCM2835Pins[iReset];
     iLEDPin = ucBCM2835Pins[iLED];
     if (iDCPin == 0 || iResetPin == 0 || iLEDPin == 0)
     {
        printf("One or more invalid GPIO Pin numbers\n");
        return -1;
     }
     if (!bcm2835_init())
	{
	printf("failed to initialize BCM2835 library\n");
        return -1;
	}
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
    if (iSPIFreq >= 32000000)
       bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_4); // 32Mhz
    else if (iSPIFreq >= 16000000)
       bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_8); // 16Mhz
    else
       bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_16); // 8Mhz

    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default
    file_spi = 0;
#endif // USE_BCM2835

#ifdef USE_PIGPIO
	iDCPin = ucPIGPins[iDC];
	iResetPin = ucPIGPins[iReset];
	iLEDPin = ucPIGPins[iLED];
	if (iDCPin == 0xff || iResetPin == 0xff || iLEDPin == 0xff)
	{
		printf("One or more invalid GPIO pin numbers\n");
		return -1;
	}
        if (gpioInitialise() < 0)
                printf("pigpio failed to initialize\n");
        file_spi = spiOpen(iChannel, iSPIFreq, 0);
#endif // USE_PIGPIO

#ifdef USE_WIRINGPI
	iDCPin = ucWPPins[iDC];
	iResetPin = ucWPPins[iReset];
	iLEDPin = ucWPPins[iLED];

	if (iDCPin == 0xff || iResetPin == 0xff || iLEDPin == 0xff)
	{
		printf("One or more invalid GPIO pin numbers\n");
		return -1;
	}
        wiringPiSetup(); // initialize GPIO interface
	file_spi = wiringPiSPISetup(iChannel, iSPIFreq); // Initialize SPI channel
#endif
	if (file_spi < 0)
	{
		fprintf(stderr, "Failed to open the SPI bus\n");
		return -1;
	}

#ifdef USE_BCM2835
	bcm2835_gpio_fsel(iDCPin, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(iResetPin, BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_fsel(iLEDPin, BCM2835_GPIO_FSEL_OUTP);
#endif
#ifdef USE_PIGPIO
	gpioSetMode(iDCPin, PI_OUTPUT);
	gpioSetMode(iResetPin, PI_OUTPUT);
	gpioSetMode(iLEDPin, PI_OUTPUT); 
#endif

#ifdef USE_WIRINGPI
        pinMode(iDCPin, OUTPUT);
	pinMode(iResetPin, OUTPUT);
	pinMode(iLEDPin, OUTPUT);
#endif

	myPinWrite(iResetPin, 1);
	usleep(100000);
	myPinWrite(iResetPin, 0); // reset the controller
	usleep(100000);
	myPinWrite(iResetPin, 1);
	usleep(200000);
	myPinWrite(iLEDPin, 1); // turn on the backlight


	spilcdWriteCommand(0x01); // software reset
	usleep(120000);

	spilcdWriteCommand(0x11);
	usleep(250000);
	if (iLCDType == LCD_HX8357)
	{
		spilcdWriteCommand(0xb0);
		spilcdWriteData16(0x00FF);
		spilcdWriteData16(0x0001);
		usleep(100000);
	}
    // Send the commands/parameters to initialize the LCD controller
	if (iLCDType == LCD_ILI9341)
	{
		s = uc240InitList;
		iWidth = 240;
		iHeight = 320;
	}
	else if (iLCDType == LCD_HX8357)
	{
		s = uc480InitList;
		iWidth = 320;
		iHeight = 480;
	}
	else // ST7735
	{
		s = uc128InitList;
		iWidth = 128;
		iHeight = 160;
	}
	iCount  = 1;
	while (iCount)
	{
		iCount = *s++;
		if (iCount != 0)
		{
			spilcdWriteCommand(*s++);
			for(i=0; i<iCount-1; i++)
			{
				spilcdWriteData8(*s++);
			}
		}
	}

	spilcdWriteCommand(0x11); // sleep out
	usleep(120000);
	spilcdWriteCommand(0x29); // Display ON
	usleep(10000);

	spilcdFill(0); // erase memory

	return 0;

} /* spilcdInit() */

// Configure a GPIO pin for input
// Returns 0 if successful, -1 if unavailable
// all input pins are assumed to use internal pullup resistors
// and are connected to ground when pressed
//
int spilcdConfigurePin(int iPin)
{
int iGPIO;

#ifdef USE_BCM2835
        iGPIO = ucBCM2835Pins[iPin];
        if (iGPIO == 0) // invalid pin number
                return -1;
        bcm2835_gpio_fsel(iGPIO, BCM2835_GPIO_FSEL_INPT);
        bcm2835_gpio_set_pud(iGPIO,  BCM2835_GPIO_PUD_UP);
#endif
#ifdef USE_PIGPIO
        iGPIO = ucPIGPins[iPin];
        if (iGPIO == 0xff) // invalid pin
                return -1;
        gpioSetMode(iGPIO, PI_INPUT);
        gpioSetPullUpDown(iGPIO, PI_PUD_UP);
#endif

#ifdef USE_WIRINGPI
        iGPIO = ucWPPins[iPin];
        if (iGPIO == 0xff) // invalid
                return -1;
        pinMode(iGPIO, INPUT);
        pullUpDnControl(iGPIO, PUD_UP);
#endif

        return 0;
} /* spilcdConfigurePin() */

// Read from a GPIO pin
int spilcdReadPin(int iPin)
{
int iGPIO;
#ifdef USE_PIGPIO
        iGPIO = ucPIGPins[iPin];
        return gpioRead(iGPIO);
#endif // USE_PIGPIO
#ifdef USE_WIRINGPI
        iGPIO = ucWPPins[iPin];
        return (digitalRead(iGPIO) == HIGH);
#endif // USE_WIRINGPI
#ifdef USE_BCM2835
        iGPIO = ucBCM2835Pins[iPin];
        return (bcm2835_gpio_lev(iGPIO) == HIGH);
#endif // USE_BCM2835
} /* spilcdReadPin() */

//
// Reset the scroll position to 0
//
void spilcdScrollReset(void)
{
	iScrollOffset = 0;
	spilcdWriteCommand(0x37); // scroll start address
	spilcdWriteData16(0);
	if (iLCDType == LCD_HX8357)
	{
		spilcdWriteData16(0);
	}
} /* spilcdScrollReset() */

//
// Scroll the screen N lines vertically (positive or negative)
// The value given represents a delta which affects the current scroll offset
// If iFillColor != -1, the newly exposed lines will be filled with that color
//
void spilcdScroll(int iLines, int iFillColor)
{
	iScrollOffset = (iScrollOffset + iLines) % iHeight;
	spilcdWriteCommand(0x37); // Vertical scrolling start address
	if (iLCDType == LCD_ILI9341 || iLCDType == LCD_ST7735)
	{
		spilcdWriteData16(iScrollOffset);
	}
	else
	{
		spilcdWriteData16(iScrollOffset >> 8);
		spilcdWriteData16(iScrollOffset & 0xff);
	}
	if (iFillColor != -1) // fill the exposed lines
	{
	int i, iStart;
	uint16_t usTemp[320];
	uint32_t *d;
	uint32_t u32Fill;
		// quickly prepare a full line's worth of the color
		u32Fill = (iFillColor >> 8) | ((iFillColor & 0xff) << 8);
		u32Fill |= (u32Fill << 16);
		d = (uint32_t *)&usTemp[0];
		for (i=0; i<iWidth/2; i++)
			*d++ = u32Fill;
		if (iLines < 0)
		{
			iStart = 0;
			iLines = 0 - iLines;
		}
		else
			iStart = iHeight - iLines;
		if (iOrientation == LCD_ORIENTATION_LANDSCAPE)
			spilcdSetPosition(iStart, iWidth-1, iLines, iWidth);
		else
			spilcdSetPosition(0, iStart, iWidth, iLines);
		for (i=0; i<iLines; i++)
		{
			spilcdWriteDataBlock((unsigned char *)usTemp, iWidth*2);
		}
	}

} /* spilcdScroll() */

void spilcdRectangle(int x, int y, int w, int h, unsigned short usColor, int bFill)
{
unsigned char ucTemp[960]; // max length
uint32_t u32Color, *pu32;
int i, iPerLine, iStart;
int trueY, trueH, trueX, trueW;

	// check bounds
	if (iOrientation == LCD_ORIENTATION_PORTRAIT)
	{
		if (x < 0 || x >= iWidth || x+w >= iWidth)
			return; // out of bounds
		if (y < 0 || y >= iHeight || y+h >= iHeight)
			return;
		trueX = x;
		trueY = y;
		trueW = w;
		trueH = h;
	}
	else
	{
		if (y < 0 || y >= iWidth || y+h >= iWidth)
			return;
		if (x < 0 || x >= iHeight || x+w >= iHeight)
			return;
		trueY = x;
		trueH = w;
		trueX = iWidth - x - 1;
		trueW = h;
	}
	u32Color = usColor >> 8;
	u32Color |= (usColor & 0xff) << 8;
	u32Color |= (u32Color << 16);
	pu32 = (uint32_t *)&ucTemp[0];
	for (i=0; i<240; i++) // prepare big buffer of color
		*pu32++ = u32Color;

	if (bFill)
	{
		iPerLine = trueW*2; // bytes to write per line
	       	spilcdSetPosition(trueX, trueY, trueW, trueH);
	        if (((trueY + iScrollOffset) % iHeight) > iHeight-trueH) // need to write in 2 parts since it won't wrap
		{
                iStart = (iHeight - ((trueY+iScrollOffset) % iHeight));
		for (i=0; i<iStart; i++)
                	spilcdWriteDataBlock(ucTemp, iStart*iPerLine); // first N lines
			spilcdSetPosition(trueX, trueY+iStart, trueW, trueH-iStart);
			for (i=0; i<trueH-iStart; i++)
               	 		spilcdWriteDataBlock(ucTemp, iPerLine);
       		 }
        	else // can write in one shot
        	{
			for (i=0; i<trueH; i++)
               		 	spilcdWriteDataBlock(ucTemp, iPerLine);
        	}
	}
	else // outline
	{
		// draw top/bottom
		spilcdSetPosition(trueX, trueY, trueW, 1);
		spilcdWriteDataBlock(ucTemp, trueW*2);
		spilcdSetPosition(trueX, trueY + trueH-1, trueW, 1);
		spilcdWriteDataBlock(ucTemp, trueW*2);
		// draw left/right
		if (((trueY + iScrollOffset) % iHeight) > iHeight-h)	
		{
			iStart = (iHeight - ((trueY+iScrollOffset) % iHeight));
			spilcdSetPosition(trueX, trueY, 1, iStart);
			spilcdWriteDataBlock(ucTemp, iStart*2);
			spilcdSetPosition(trueX+trueW-1, trueY, 1, iStart);
			spilcdWriteDataBlock(ucTemp, iStart*2);
			// second half
			spilcdSetPosition(trueX,trueY+iStart, 1, trueH-iStart);
			spilcdWriteDataBlock(ucTemp, (trueH-iStart)*2);
			spilcdSetPosition(trueX+trueW-1, trueY+iStart, 1, trueH-iStart);
			spilcdWriteDataBlock(ucTemp, (trueH-iStart)*2);
		}
		else // can do it in 1 shot
		{
			spilcdSetPosition(trueX, trueY, 1, trueH);
			spilcdWriteDataBlock(ucTemp, trueH*2);
			spilcdSetPosition(trueX + trueW-1, trueY, 1, trueH);
			spilcdWriteDataBlock(ucTemp, trueH*2);
		}
	} // outline
} /* spilcdRectangle() */

//
// Sends a command to turn off the LCD display
// Turns off the backlight LED
// Closes the SPI file handle
//
void spilcdShutdown(void)
{
	if (file_spi >= 0)
	{
		spilcdWriteCommand(0x29); // Display OFF
#ifdef USE_PIGPIO
		spiClose(file_spi);
#endif
#ifdef USE_WIRINGPI
		close(file_spi);
#endif // USE_WIRINGPI

		file_spi = -1;
		myPinWrite(iLEDPin, 0); // turn off the backlight
#ifdef USE_PIGPIO
		gpioTerminate();
#endif
#ifdef USE_BCM2835
        bcm2835_spi_end();
        bcm2835_close();
#endif

	}
} /* spilcdShutdown() */

//
// Send a command byte to the LCD controller
// In SPI 8-bit mode, the D/C line must be set
// high during the write
//
static void spilcdWriteCommand(unsigned char c)
{
unsigned char buf[2];

	spilcdSetMode(MODE_COMMAND);
	buf[0] = c;
	myspiWrite(buf, 1);
	spilcdSetMode(MODE_DATA);
} /* spilcdWriteCommand() */

//
// Write a single byte of data
//
static void spilcdWriteData8(unsigned char c)
{
unsigned char buf[2];

	buf[0] = c;
    myspiWrite(buf, 1);

} /* spilcdWriteData8() */

//
// Write 16-bits of data
// The ILI9341 receives data in big-endian order
// (MSB first)
//
static void spilcdWriteData16(unsigned short us)
{
unsigned char buf[2];

    buf[0] = (unsigned char)(us >> 8);
    buf[1] = (unsigned char)us;
    myspiWrite(buf, 2);

} /* spilcdWriteData16() */

//
// Position the "cursor" to the given
// row and column. The width and height of the memory
// 'window' must be specified as well. The controller
// allows more efficient writing of small blocks (e.g. tiles)
// by bounding the writes within a small area and automatically
// wrapping the address when reaching the end of the window
// on the current row
//
static void spilcdSetPosition(int x, int y, int w, int h)
{
unsigned char ucBuf[8];
int t;

	if (iOrientation == LCD_ORIENTATION_LANDSCAPE) // rotate 90 clockwise
	{
		// rotate the coordinate system
		t = x;
		x = iWidth-1-y;
		y = t;
		// flip the width/height too
		t = w;
		w = h;
		h = t;
	}
	y = (y + iScrollOffset) % iHeight; // scroll offset affects writing position

	spilcdWriteCommand(0x2a); // set column address
	if (iLCDType == LCD_ILI9341 || iLCDType == LCD_ST7735)
	{
		ucBuf[0] = (unsigned char)(x >> 8);
		ucBuf[1] = (unsigned char)x;
		x = x + w - 1;
		if (x > iWidth-1) x = iWidth-1;
		ucBuf[2] = (unsigned char)(x >> 8);
		ucBuf[3] = (unsigned char)x; 
		myspiWrite(ucBuf, 4);
	}
	else
	{
// combine coordinates into 1 write to save time
		ucBuf[0] = 0;
 		ucBuf[1] = (unsigned char)(x >> 8); // MSB first
		ucBuf[2] = 0;
		ucBuf[3] = (unsigned char)x;
		x = x + w -1;
		if (x > iWidth-1) x = iWidth-1;
		ucBuf[4] = 0;
		ucBuf[5] = (unsigned char)(x >> 8);
		ucBuf[6] = 0;
		ucBuf[7] = (unsigned char)x;
		myspiWrite(ucBuf, 8); 
	}
	spilcdWriteCommand(0x2b); // set row address
	if (iLCDType == LCD_ILI9341 || iLCDType == LCD_ST7735)
	{
		ucBuf[0] = (unsigned char)(y >> 8);
		ucBuf[1] = (unsigned char)y;
		y = y + h - 1;
		if (y > iHeight-1) y = iHeight-1;
		ucBuf[2] = (unsigned char)(y >> 8);
		ucBuf[3] = (unsigned char)y;
		myspiWrite(ucBuf, 4);
	}
	else
	{
// combine coordinates into 1 write to save time
		ucBuf[0] = 0;
		ucBuf[1] = (unsigned char)(y >> 8); // MSB first
		ucBuf[2] = 0;
		ucBuf[3] = (unsigned char)y;
		y = y + h - 1;
		if (y > iHeight-1) y = iHeight-1;
		ucBuf[4] = 0;
		ucBuf[5] = (unsigned char)(y >> 8);
		ucBuf[6] = 0;
		ucBuf[7] = (unsigned char)y;
		myspiWrite(ucBuf, 8);
	}
	spilcdWriteCommand(0x2c); // write memory begin
//	spilcdWriteCommand(0x3c); // write memory continue
} /* spilcdSetPosition() */

//
// Write a block of pixel data to the LCD
// Each pixel (16-bits) must be in big-endian order
// The internal address automatically increments as
// each byte is received. This allows efficient writing
// of characters/tiles by setting the appropriate window size
// and doing the write in one shot.
//
static void spilcdWriteDataBlock(unsigned char *ucBuf, int iLen)
{
        myspiWrite(ucBuf, iLen);
} /* spilcdWriteDataBlock() */

//
// Draw a 16x16 tile as 16x14 (with pixel averaging)
// This is for drawing 160x144 video games onto a 160x128 display
// It is assumed that the display is set to LANDSCAPE orientation
//
int spilcdDrawSmallTile(int x, int y, unsigned char *pTile, int iPitch)
{
unsigned char ucTemp[448];
int i, j, iPitch32;
uint16_t *d;
uint32_t *s;
uint32_t u32A, u32B, u32a, u32b, u32C, u32D;
uint32_t u32Magic = 0xf7def7de;
uint32_t u32Mask = 0xffff;

        if (file_spi < 0) return -1;

        // scale y coordinate for shrinking
        y = (y * 7)/8;
        iPitch32 = iPitch/4;
        for (j=0; j<16; j+=2) // 16 source lines (2 at a time)
        {
                s = (uint32_t *)&pTile[j * 2];
                d = (uint16_t *)&ucTemp[j*28];
                for (i=0; i<16; i+=2) // 16 source columns (2 at a time)
                {
                        u32A = s[(15-i)*iPitch32]; // read A+C
                        u32B = s[(14-i)*iPitch32]; // read B+D
                        u32C = u32A >> 16;
                        u32D = u32B >> 16;
                        u32A &= u32Mask;
                        u32B &= u32Mask;
			if (i == 0 || i == 8) // pixel average a pair
			{
                        	u32a = (u32A & u32Magic) >> 1;
                        	u32a += ((u32B & u32Magic) >> 1);
                        	u32b = (u32C & u32Magic) >> 1;
                        	u32b += ((u32D & u32Magic) >> 1);
				d[0] = __builtin_bswap16(u32a);
				d[14] = __builtin_bswap16(u32b);
				d++;
			}
			else
			{
                        	d[0] = __builtin_bswap16(u32A);
                        	d[1] = __builtin_bswap16(u32B);
                        	d[14] = __builtin_bswap16(u32C);
                        	d[15] = __builtin_bswap16(u32D);
                       		d += 2;
			}
                } // for i
        } // for j
        spilcdSetPosition(x, y+13, 16, 14); // since RAM is oriented diff, adjust y
        if (((x + iScrollOffset) % iHeight) > iHeight-16) // need to write in 2 parts since it won't wrap
        {
                int iStart = (iHeight - ((x+iScrollOffset) % iHeight));
                spilcdWriteDataBlock(ucTemp, iStart*28); // first N lines
                spilcdSetPosition(x+iStart, y+13, 16-iStart, 14);
                spilcdWriteDataBlock(&ucTemp[iStart*28], 448-(iStart*28));
        }
        else // can write in one shot
        {
                spilcdWriteDataBlock(ucTemp, 448);
	}
	return 0;
} /* spilcdDrawSmallTile() */
//
// Draw a 16x16 RGB565 tile scaled to 32x24
// The main purpose of this function is for GameBoy emulation
// Since the original display is 160x144, this function allows it to be 
// stretched 100x50% larger (320x216). Not a perfect fit for 320x240, but better
// Each group of 2x2 pixels becomes a group of 4x3 pixels by averaging the pixels
//
// +-+-+ becomes +-+-+-+-+
// |A|B|         |A|A|B|B|
// +-+-+         +-+-+-+-+
// |C|D|         |a|a|b|b| a = A avg. B, b = B avg. D
// +-+-+         +-+-+-+-+
//               |C|C|D|D|
//               +-+-+-+-+
//
// The x/y coordinates will be scaled
// It is assumed that the display is set to LANDSCAPE orientation
//
int spilcdDrawScaledTile(int x, int y, unsigned char *pTile, int iPitch)
{
unsigned char ucTemp[1536];
int i, j, iPitch32;
uint16_t *d;
uint32_t *s;
uint32_t u32A, u32B, u32a, u32b, u32C, u32D;
uint32_t u32Magic = 0xf7def7de;
uint32_t u32Mask = 0xffff;

	if (file_spi < 0) return -1;

	// scale coordinates for stretching
	x = x * 2;
	y = (y * 3)/2;
        iPitch32 = iPitch/4;
	for (j=0; j<16; j+=2) // 16 source lines (2 at a time)
	{
		s = (uint32_t *)&pTile[j * 2];
		d = (uint16_t *)&ucTemp[j*96];
		for (i=0; i<16; i+=2) // 16 source columns (2 at a time)
		{
			u32A = s[(15-i)*iPitch32];
			u32B = s[(14-i)*iPitch32];
			u32C = u32A >> 16;
			u32D = u32B >> 16;
			u32A &= u32Mask;
			u32B &= u32Mask;
			u32a = (u32A & u32Magic) >> 1;
			u32a += ((u32B & u32Magic) >> 1);
			u32b = (u32C & u32Magic) >> 1;
			u32b += ((u32D & u32Magic) >> 1);
			d[0] = d[24] = __builtin_bswap16(u32A); // swap byte order
			d[1] = d[25] = __builtin_bswap16(u32a);
			d[2] = d[26] = __builtin_bswap16(u32B);
			d[48] = d[72] = __builtin_bswap16(u32C);
			d[49] = d[73] = __builtin_bswap16(u32b);
			d[50] = d[74] = __builtin_bswap16(u32D);
			d += 3;
		} // for i
	} // for j
        spilcdSetPosition(x, y+23, 32, 24); // since RAM is oriented diff, adjust y
        if (((x + iScrollOffset) % iHeight) > iHeight-32) // need to write in 2 parts since it won't wrap
        {
                int iStart = (iHeight - ((x+iScrollOffset) % iHeight));
                spilcdWriteDataBlock(ucTemp, iStart*48); // first N lines
                spilcdSetPosition(x+iStart, y+23, 32-iStart, 24);
                spilcdWriteDataBlock(&ucTemp[iStart*48], 1536-(iStart*48));
        }
        else // can write in one shot
        {
                spilcdWriteDataBlock(ucTemp, 1536);
        }
	return 0;
} /* spilcdDrawScaledTile() */

//
// Draw a 16x16 RGB565 tile
// This reverses the pixel byte order and sets a memory "window"
// of 16x16 pixels so that the write can occur in one shot
//
int spilcdDrawTile(int x, int y, unsigned char *pTile, int iPitch)
{
unsigned char ucTemp[512]; // fix the byte order first to write it more quickly
int i, j;
unsigned char *s, *d;

	if (file_spi < 0) return -1;

	if (iOrientation == LCD_ORIENTATION_LANDSCAPE) // need to rotate the data
	{
        	// First convert to big-endian order
        	d = ucTemp;
        	for (j=0; j<16; j++)
        	{
                	s = &pTile[j*2];
                	for (i=0; i<16; i++)
                	{
                        	d[1] = s[(15-i)*iPitch];
                        	d[0] = s[((15-i)*iPitch)+1]; // swap byte order (MSB first)
                        	d += 2;
                	} // for i;
        	} // for j
        	spilcdSetPosition(x, y+15, 16, 16); // since RAM is oriented diff, adjust y
		if (((x + iScrollOffset) % iHeight) > iHeight-16) // need to write in 2 parts since it won't wrap
		{
			int iStart = (iHeight - ((x+iScrollOffset) % iHeight));
			spilcdWriteDataBlock(ucTemp, iStart*32); // first N lines
			spilcdSetPosition(x+iStart, y+15, 16-iStart, 16); 
			spilcdWriteDataBlock(&ucTemp[iStart*32], 512-(iStart*32));
		}
		else // can write in one shot
		{
			spilcdWriteDataBlock(ucTemp, 512);
		}
	}
	else // native orientation
	{
	// First convert to big-endian order
	d = ucTemp;
	for (j=0; j<16; j++)
	{
		s = &pTile[j*iPitch];
		for (i=0; i<16; i++)
		{
			d[1] = s[0];
			d[0] = s[1]; // swap byte order (MSB first)
			d += 2;
			s += 2;
		} // for i;
	} // for j
	spilcdSetPosition(x, y, 16, 16);
	if (((y + iScrollOffset) % iHeight) > iHeight-16) // need to write in 2 parts since it won't wrap
        {
                int iStart = (iHeight - ((y+iScrollOffset) % iHeight));
                spilcdWriteDataBlock(ucTemp, iStart*32); // first N lines
                spilcdSetPosition(x, y+iStart, 16, 16-iStart);
                spilcdWriteDataBlock(&ucTemp[iStart*32], 512-(iStart*32));
        }
        else // can write in one shot
	{
        	spilcdWriteDataBlock(ucTemp, 512);
	}
	} // portrait orientation
	return 0;
} /* spilcdDrawTile() */

//
// Draw an individual RGB565 pixel
//
int spilcdSetPixel(int x, int y, unsigned short usColor)
{

	if (file_spi < 0)
		return -1;

	spilcdSetPosition(x, y, 1, 1);
	spilcdWriteData16(usColor);
	return 0;
} /* spilcdSetPixel() */

//
// Draw a string of small (8x8) or large (16x32) characters
// At the given col+row
//
int spilcdWriteString(int x, int y, char *szMsg, unsigned short usFGColor, unsigned short usBGColor, int bLarge)
{
int i, j, k, iMaxLen, iLen;
unsigned char *s;
unsigned short usFG = (usFGColor >> 8) | ((usFGColor & 0xff)<< 8);
unsigned short usBG = (usBGColor >> 8) | ((usBGColor & 0xff)<< 8);


	if (file_spi < 0) return -1; // not initialized

	iLen = strlen(szMsg);
	iMaxLen = (iOrientation == LCD_ORIENTATION_PORTRAIT) ? iWidth : iHeight;

	if (bLarge) // draw 16x32 font
	{
		if (iLen*16 + x > iMaxLen) iLen = (iMaxLen - x) / 16;
		if (iLen < 0) return -1;
		for (i=0; i<iLen; i++)
		{
			unsigned short usTemp[512];
			unsigned short *usD;

			s = &ucFont[9728 + (unsigned char)szMsg[i]*64];
			usD = &usTemp[0];
			if (iOrientation == LCD_ORIENTATION_LANDSCAPE) // rotated
			{
				spilcdSetPosition(x+(i*16), y+31,16,32);
				for (j=0; j<8; j++)
				{
					for (k=0; k<32; k++) // for each scanline
					{ // left half
						if (s[62-(k*2)] & (0x80>>j))
							*usD++ = usFG;
						else
							*usD++ = usBG;
					} // for k
				} // for j
				// right half
				for (j=0; j<8; j++)
				{
					for (k=0; k<32; k++)
					{
						if (s[63-(k*2)] & (0x80>>j))
							*usD++ = usFG;
						else
							*usD++ = usBG;
					} // for k
				} // for j
			}
			else
			{ // portrait
				spilcdSetPosition(x+(i*16), y,16,32);
				for (k=0; k<32; k++) // for each scanline
				{ // left half
					for (j=0; j<8; j++)
					{
						if (s[0] & (0x80>>j))
							*usD++ = usFG;
						else
							*usD++ = usBG;
					} // for j
				// right half
					for (j=0; j<8; j++)
					{
						if (s[1] & (0x80>>j))
							*usD++ = usFG;
						else
							*usD++ = usBG;
					} // for j
					s += 2;
				} // for each scanline
			} // portrait mode
			spilcdWriteDataBlock((unsigned char *)usTemp, 1024);
		} // for each character
	}
	else // draw 8x8 font
	{
		unsigned short usTemp[64];
		unsigned short *usD;

		if ((8*iLen) + x > iMaxLen) iLen = (iMaxLen - x)/8; // can't display it all
		if (iLen < 0)return -1;

		for (i=0; i<iLen; i++)
		{
			s = &ucFont[(unsigned char)szMsg[i] * 8];
			usD = &usTemp[0];
			if (iOrientation == LCD_ORIENTATION_LANDSCAPE) // draw rotated
			{
				spilcdSetPosition(x+(i*8), y+7, 8, 8);
				for (k=0; k<8; k++) // for each scanline
				{
					for (j=0; j<8; j++)
					{
						if (s[7-j] & (0x80 >> k))
							*usD++ = usFG;
						else
							*usD++ = usBG;	
					} // for j
				} // for k
			}
			else // portrait orientation
			{
				spilcdSetPosition(x+(i*8), y, 8, 8);
				for (k=0; k<8; k++) // for each scanline
				{
					for (j=0; j<8; j++)
					{
						if (s[0] & (0x80>>j))
							*usD++ = usFG;
						else
							*usD++ = usBG;
					} // for j
					s++;
				} // for k
			} // normal orientation
		// write the data in one shot
			spilcdWriteDataBlock((unsigned char *)usTemp, 128);
		}	
	}
	return 0;
} /* spilcdWriteString() */

//
// Set the (software) orientation of the display
// The hardware is permanently oriented in 240x320 portrait mode
// The library can draw characters/tiles rotated 90
// degrees if set into landscape mode
//
int spilcdSetOrientation(int iOrient)
{
	if (iOrient != LCD_ORIENTATION_PORTRAIT && iOrient != LCD_ORIENTATION_LANDSCAPE)
		return -1;
	iOrientation = iOrient; // nothing else needed to do
	return 0;
} /* spilcdSetOrientation() */

//
// Fill the frame buffer with a single color
//
int spilcdFill(unsigned short usData)
{
int y;
unsigned short usC, temp[320];
int iOldOrient;

	if (file_spi < 0) return -1; // not initialized

	usC = (usData >> 8) | ((usData & 0xff)<<8); // swap endian-ness
	// make sure we're in landscape mode to use the correct coordinates
	iOldOrient = iOrientation;
	iOrientation = LCD_ORIENTATION_PORTRAIT;
	spilcdScrollReset();
	spilcdSetPosition(0,0,iWidth,iHeight);
	iOrientation = iOldOrient;

	for (y=0; y<iWidth; y++)
           temp[y] = usC;
	for (y=0; y<iHeight; y++)
	{
		spilcdWriteDataBlock((unsigned char *)temp, iWidth*2); // fill with data byte
	} // for y
	return 0;
} /* spilcdFill() */
