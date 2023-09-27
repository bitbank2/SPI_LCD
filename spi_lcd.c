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

// Use one of the following 4 methods for talking to the SPI/GPIO
#define USE_PIGPIO
//#define USE_BCM2835
//#define USE_WIRINGPI
//#define USE_GENERIC

// For generic SPI access (kernel drivers), select the board pinout (only one)
//#define USE_NANOPI2
//#define USE_NANOPIK2
//#define USE_NANOPIDUO
//#define USE_NANOPINEO
//define USE_NANOPIM1
//#define USE_RPI
//#define USE_ORANGEPIZERO
//#define USE_ORANGEPIONE
//#define USE_BANANAPIM2ZERO
//#define USE_BANANAPIM2MAGIC
//#define USE_NANOPINEOCORE
//#define USE_ORANGEPIZEROPLUS2

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
#ifdef __NEON__
#include <arm_neon.h>
#endif //__NEON__

#ifdef USE_GENERIC
#include <linux/spi/spidev.h>
static struct spi_ioc_transfer xfer;
#define GPIO_OUT 0
#define GPIO_IN 1
static int iPinHandles[256]; // keep file handles open for GPIO access
#endif // USE_GENERIC

extern unsigned char ucFont[];
static unsigned char ucRXBuf[4096]; //, ucRXBuf2[4096];
static int file_spi = -1; // SPI system handle
static int file_touch = -1; // SPI handle for touch controller
static int iTouchChannel, iTouchType;
static int iDCPin, iResetPin, iLEDPin; // pin numbers for the GPIO control lines
static char iMinX;
static int iMinXOLD, iMaxX, iMinY, iMaxY; // touch calibration values
static int iScrollOffset; // current scroll amount
static int iOrientation = LCD_ORIENTATION_NATIVE; // default to 'natural' orientation
static int iLCDType;
static int iWidth, iHeight;
static int iCurrentWidth, iCurrentHeight; // reflects virtual size due to orientation

static void spilcdWriteCommand(unsigned char);
static void spilcdWriteData8(unsigned char c);
static void spilcdWriteData16(unsigned short us);
static void spilcdSetPosition(int x, int y, int w, int h);
static void spilcdWriteDataBlock(unsigned char *pData, int iLen);
static void myPinWrite(int iPin, int iValue);
int spilcdFill(unsigned short usData);

// For Raspberry Pi boards, we can use the generic GPIO/SPI access too
#ifdef USE_RPI
static int iGenericPins[] = {-1,-1,-1,2,-1,3,-1,4,14,-1,
			15,17,18,27,-1,22,23,-1,24,10,
			-1,9,25,11,8,-1,7,0,1,5,
			-1,6,12,13,-1,19,16,26,20,-1,
			21};
#endif // USE_RPI

#ifdef USE_NANOPINEOCORE
static int iGenericPins[] = {-1,-1,-1,12,-1,11,-1,203,198,-1,
                        199,0,6,2,-1,3,200,-1,201,64,
                        -1,65,1,66,67,-1,-1,-1,-1,-1,
                        -1,-1,-1,-1,-1,-1,-1,-1,167,-1,
                        140,-1,141,-1,-1,-1,15,-1,16,-1,
                        14,-1,13,-1,-1,363,-1,17,-1,18,
                        -1,19,164,20,162,21,-1,-1,-1};
#endif // USE_NANOPINEOCORE

#ifdef USE_ORANGEPIZEROPLUS2
static int iGenericPins[] = {-1,-1,-1,12,-1,11,-1,6,0,-1,1,352,107,353,-1,3,
	19,-1,18,-1,-1,-1,2,14,13,-1,110,-1,5,4,-1,-1,-1,-1,-1,-1,-1,-1,-1,
	-1,-1};
#endif // USE_ORANGEPIZEROPLUS2

#ifdef USE_BANANAPIM2ZERO
static int iGenericPins[] = {-1,-1,-1,12,-1,11,-1,6,13,-1,14,1,110,0,-1,3,15,-1,68,64,-1,65,2,66,67,-1,71,19,18,7,-1,8,354,9,-1,10,356,17,21,-1,20};
#endif // BPI-M2-Zero

#ifdef USE_BANANAPIM2MAGIC
static int iGenericPins[] = {-1,-1,-1,-1,-1,-1,-1,225,32,-1,33,230,203,231,-1,117,34,-1,35,64,-1,65,116,66,67,-1,121,115,120,114,-1,119,118,123,-1,202,122,363,205,-114,204};
#endif // BPI-M2-Magic

#ifdef USE_ORANGEPIONE
static int iGenericPins[] = {-1,-1,-1,12,-1,11,-1,6,13,-1,14,1,110,0,-1,3,68,-1,71,64,-1,65,2,66,67,-1,21,19,18,7,-1,8,200,9,-1,10,201,20,198,-1,199};
#endif // ORANGEPIONE

#ifdef USE_ORANGEPIZERO
static int iGenericPins[] = {-1,-1,-1,12,-1,11,-1,6,198,-1,199,1,7,0,-1,3,19,-1,18,15,-1,16,2,14,13,-1,10,-1,5,4,-1,
-1,-1,-1,-1,-1,-1,-1,-1};
#endif // ORANGEPIZERO

#ifdef USE_NANOPIDUO
static int iGenericPins[] = {-1,5,-1,4,-1,-1,-1,11,
			-1,12,363,13,203,14,-1,16,
			-1,15,-1,199,-1,198,-1,-1,
			-1,-1,-1,-1,-1,-1,-1,-1,
			-1,355,-1,-1,-1,-1,-1,-1,-1};
#endif // USE_NANOPIDUO

#ifdef USE_NANOPI2
static int iGenericPins[] = {-1,-1,-1,99,-1,98,-1,32+28,96+21,-1,96+17,
                       32+29,32+26,32+30,-1,32+31,64+14,-1,32+27,64+31,-1,96+0,96+1,64+29,64+30,-1,64+13,103,102,
                       64+8,-1,64+9,64+28,64+10,-1,64+12,64+7,64+11,162,-1,163};

#endif // USE_NANOPI2

#ifdef USE_NANOPIM1
static int iGenericPins[] = {-1,-1,-1,12,-1,11,-1,203,198,-1,199,0,6,2,-1,3,200,-1,201,64,-1,65,1,66,67,-1,17,19,18,20,-1,21,7,8,-1,16,13,9,15,-1,14};
#endif // USE_NANOPIM1

#ifdef USE_NANOPIK2
static int iGenericPins[] = {-1,-1,-1,205,-1,206,-1,211,102,-1,225,212,227,213,-1,214,226,-1,215,216,-1,218,217,220,219,-1,221,207,208,222,-1,127,223,155,-1,252,-1,-1,-1,-1,-1};
#endif // USE_NANOPIK2

#ifdef USE_NANOPINEO
// NanoPi NEO
// define 40 pins since the 12 pin header has 2 GPIOs available and so does
// the 4-pin TTY header
static int iGenericPins[] = {-1,-1,-1,12,-1,11,-1,203,198,-1,
                        199,0,6,2,-1,3,200,-1,201,64,
                        -1,65,1,66,67,-1,-1,-1,-1,-1,
                        363,17,-1,-1,-1,-1,-1,-1,-1,4,
                        5};
#endif // USE_NANOPINEO
#ifdef USE_PIGPIO
static int iPIGPins[] = {-1,-1,-1,2,-1,3,-1,4,14,-1,15,
                       17,18,27,-1,22,23,-1,24,10,-1,9,25,11,8,-1,7,0,1,
                       5,-1,6,12,13,-1,19,16,26,20,-1,21};
#endif // USE_PIGPIO

#ifdef USE_WIRINGPI
static int iWPPins[] = {-1,-1,-1,8,-1,9,-1,7,15,-1,16,0,1,
        2,-1,3,4,-1,5,12,-1,13,6,14,10,-1,11,30,31,21,-1,22,26,23,-1,24,27,25,28,-1,29};
#endif // USE_WIRINGPI

#ifdef USE_BCM2835
static int iBCM2835Pins[] = {-1,-1,-1,RPI_V2_GPIO_P1_03,-1,RPI_V2_GPIO_P1_05,-1,
	RPI_V2_GPIO_P1_07, RPI_V2_GPIO_P1_08,-1, RPI_V2_GPIO_P1_10, RPI_V2_GPIO_P1_11,
        RPI_V2_GPIO_P1_12, RPI_V2_GPIO_P1_13, -1, RPI_V2_GPIO_P1_15,RPI_V2_GPIO_P1_16,
        -1, RPI_V2_GPIO_P1_18, RPI_V2_GPIO_P1_19, -1, RPI_V2_GPIO_P1_21, RPI_V2_GPIO_P1_22,
        RPI_V2_GPIO_P1_23, RPI_V2_GPIO_P1_24, -1, RPI_V2_GPIO_P1_26, -1, -1,
        RPI_V2_GPIO_P1_29, -1, RPI_V2_GPIO_P1_31, RPI_V2_GPIO_P1_32, RPI_V2_GPIO_P1_33,
        -1, RPI_V2_GPIO_P1_35, RPI_V2_GPIO_P1_36, RPI_V2_GPIO_P1_37, RPI_V2_GPIO_P1_38,
        -1, RPI_V2_GPIO_P1_40};
#endif // BCM2835

// Sets the D/C pin to data or command mode
void spilcdSetMode(int iMode)
{
	myPinWrite(iDCPin, iMode == MODE_DATA);
} /* spilcdSetMode() */

// List of command/parameters to initialize the SSD1351 OLED display
static unsigned char ucOLEDInitList[] = {
	2, 0xfd, 0x12, // unlock the controller
	2, 0xfd, 0xb1, // unlock the command
	1, 0xae,	// display off
	2, 0xb3, 0xf1,  // clock divider
	2, 0xca, 0x7f,	// mux ratio
	2, 0xa0, 0x74,	// set remap
	3, 0x15, 0x00, 0x7f,	// set column
	3, 0x75, 0x00, 0x7f,	// set row
	2, 0xb5, 0x00,	// set GPIO state
	2, 0xab, 0x01,	// function select (internal diode drop)
	2, 0xb1, 0x32,	// precharge
	2, 0xbe, 0x05,	// vcomh
	1, 0xa6,	// set normal display mode
	4, 0xc1, 0xc8, 0x80, 0xc8, // contrast ABC
	2, 0xc7, 0x0f,	// contrast master
	4, 0xb4, 0xa0,0xb5,0x55,	// set VSL
	2, 0xb6, 0x01,	// precharge 2
	1, 0xaf,	// display ON
	0};
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
// List of command/parameters to initialize the ili9342 display
static unsigned char uc320InitList[] = {
        2, 0xc0, 0x23, // Power control
        2, 0xc1, 0x10, // Power control
        3, 0xc5, 0x3e, 0x28, // VCM control
        2, 0xc7, 0x86, // VCM control2
        2, 0x36, 0x08, // Memory Access Control (flip x/y/bgr/rgb)
        2, 0x3a, 0x55,
	1, 0x21,	// inverted display off
//        2, 0x26, 0x01, // Gamma curve selected
//        16, 0xe0, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08,
//                0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00, // Set Gamma
//        16, 0xe1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07,
//                0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F, // Set Gamma
//        3, 0xb1, 0x00, 0x10, // FrameRate Control 119Hz
        0
};
// List of command/parameters to initialize the ST7789 LCD
static unsigned char uc240x240InitList[] = {
	1, 0x13, // partial mode off
	1, 0x21, // display inversion off
	2, 0x36,0x08,	// memory access 0xc0 for 180 degree flipped
	2, 0x3a,0x55,	// pixel format; 5=RGB565
	3, 0x37,0x00,0x00, //
	6, 0xb2,0x0c,0x0c,0x00,0x33,0x33, // Porch control
	2, 0xb7,0x35,	// gate control
	2, 0xbb,0x1a,	// VCOM
	2, 0xc0,0x2c,	// LCM
	2, 0xc2,0x01,	// VDV & VRH command enable
	2, 0xc3,0x0b,	// VRH set
	2, 0xc4,0x20,	// VDV set
	2, 0xc6,0x0f,	// FR control 2
	3, 0xd0, 0xa4, 0xa1, 	// Power control 1
	15, 0xe0, 0x00,0x19,0x1e,0x0a,0x09,0x15,0x3d,0x44,0x51,0x12,0x03,
		0x00,0x3f,0x3f, 	// gamma 1
	15, 0xe1, 0x00,0x18,0x1e,0x0a,0x09,0x25,0x3f,0x43,0x52,0x33,0x03,
		0x00,0x3f,0x3f,		// gamma 2
	1, 0x29,	// display on
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
// Wrapper function for simultaneous read/write to SPI bus
//
static int myspiReadWrite(unsigned char *pTxBuf, unsigned char *pRxBuf, int iLen)
{
int i = 0;
#ifdef USE_PIGPIO
	i = spiXfer(file_touch, (char *)pTxBuf, (char *)pRxBuf, iLen);
#endif
#ifdef USE_WIRINGPI
	memcpy(pRxBuf, pTxBuf, iLen);
//	i = wiringPiSPIDataRW(iTouchChannel, pRxBuf); 
#endif
return i;
} /* myspiReadWrite() */

//
// Wrapper function for writing to SPI
//
static void myspiWrite(unsigned char *pBuf, int iLen)
{
#ifdef USE_GENERIC
	xfer.tx_buf = (unsigned long)pBuf;
	xfer.len = iLen;
	ioctl(file_spi, SPI_IOC_MESSAGE(1), &xfer);
#endif // USE_GENERIC
 
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
#ifdef USE_GENERIC
int rc;
char szTemp[64];

	if (iPinHandles[iPin] == -1) // not open yet
	{
		sprintf(szTemp, "/sys/class/gpio/gpio%d/value", iPin);
		iPinHandles[iPin] = open(szTemp, O_WRONLY);
	}
	if (iValue) rc = write(iPinHandles[iPin], "1", 1);
	else rc = write(iPinHandles[iPin], "0", 1);
	if (rc < 0) // error
	{ // do something
	}
#endif // USE_GENERIC

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
// Read the current touch values
//
// returns -1 for error, 0 for no touch info, 1 for touch info
//
int spilcdReadTouchPos(int *pX, int *pY)
{
// commands and SPI transaction filler to read 3 byte response for x/y
unsigned char ucReadX[] = {0xd0,0x00,0x00};
unsigned char ucReadY[] = {0x90,0x00,0x00};
unsigned char ucRxBuf[6];
int x, y;

	if (pX == NULL || pY == NULL)
		return -1;
	ucRxBuf[0] = ucRxBuf[1] = ucRxBuf[2] = ucRxBuf[3] = ucRxBuf[4] = ucRxBuf[5] = 0; // suppress compiler warning
	myspiReadWrite(ucReadX, ucRxBuf, 3);
	myspiReadWrite(ucReadY, &ucRxBuf[3], 3);
	x = ((ucRxBuf[2] + (ucRxBuf[1]<<8)) >> 4); // top 12 bits
	y = ((ucRxBuf[5] + (ucRxBuf[4]<<8)) >> 4);
	if (x > iMaxX) x = iMaxX;
	x -= iMinX;
	if (x < 0) x = 0;
	if (y > iMaxY) y = iMaxY;
	y -= iMinY;
	if (y < 0) y = 0;
	// normalize values to be in 0-1023 range
	x = (x << 10)/(iMaxX - iMinX);
	y = (y << 10)/(iMaxY - iMinY);
	// flip coordinate system
	x = 1023-x;
	y = 1023-y;
	if (x < 0) x = 0;
	if (y < 0) y = 0;
	*pX = x;
	*pY = y;

	ucRxBuf[6] = x * y;

	return (ucRxBuf[1] != 0 || ucRxBuf[2] != 0);
} /* spilcdreadTouchPos() */

void spilcdShutdownTouch(void)
{
#ifdef USE_PIGPIO
                spiClose(file_touch);
		gpioTerminate();
#endif
#ifdef USE_WIRINGPI
                close(file_touch);
#endif // USE_WIRINGPI

                file_touch = -1;

} /* spilcdShutdownTouch() */

//
// Set calibration values for touch input
//
void spilcdTouchCalibration(int iminx, int imaxx, int iminy, int imaxy)
{
	iMinX = iminx;
	iMaxX = imaxx;
	iMinY = iminy;
	iMaxY = imaxy;

} /* spilcdTouchCalibration() */

//
// Initialize the touch controller
//
int spilcdInitTouch(int iType, int iChannel, int iSPIFreq)
{
unsigned char ucInitString[] = {0x80,0x00,0x00};
unsigned char ucRxBuf[4];

	if (iType != TOUCH_XPT2046)
		return -1;

	iTouchChannel = iChannel;
	iTouchType = iType;
#ifdef USE_PIGPIO
	if (gpioInitialise() < 0)
	{
                printf("pigpio failed to initialize\n");
		return -1;
	}

	file_touch = spiOpen(iChannel, iSPIFreq, 0);
#endif
#ifdef USE_WIRINGPI
	 wiringPiSetup();
	 file_touch = wiringPiSPISetup(iChannel, iSPIFreq);
#endif
	myspiReadWrite(ucInitString, ucRxBuf, 4);
	return 0;

} /* spilcdInitTouch() */

#ifdef USE_GENERIC
void GenericAddGPIO(int iPin, int iDirection, int bPullup)
{
char szName[64];
int file_gpio, rc;
	file_gpio = open("/sys/class/gpio/export", O_WRONLY);
	sprintf(szName, "%d", iPin);
	rc = write(file_gpio, szName, strlen(szName));
	close(file_gpio);
	sprintf(szName, "/sys/class/gpio/gpio%d/direction", iPin);
	file_gpio = open(szName, O_WRONLY);
	if (iDirection == GPIO_OUT)
		rc = write(file_gpio, "out", 3);
	else
	{
                if (bPullup) // set output value to 1
                {
                int temp;
                        rc = write(file_gpio, "out",3);
                        sprintf(szName, "/sys/class/gpio/gpio%d/value", iPin);
                        temp = open(szName, O_WRONLY);
                        rc = write(temp, "1",1);
                        close(temp);
                }
		rc = write(file_gpio, "in", 2);
	}
	close(file_gpio);
	iPinHandles[iPin] = -1;
	if (rc < 0) // added to suppress compiler warnings
	{ // do nothing
	}
} /* GenericAddGPIO() */
void GenericRemoveGPIO(int iPin)
{
int file_gpio, rc;
char szTemp[64];

	close(iPinHandles[iPin]);
	file_gpio = open("/sys/class/gpio/unexport", O_WRONLY);
	sprintf(szTemp, "%d", iPin);
	rc = write(file_gpio, szTemp, strlen(szTemp));
	close(file_gpio);
	if (rc < 0) // suppress compiler warning
	{ // do nothing
	}
} /* GenericRemoveGPIO() */
#endif // USE_GENERIC

//
// Choose the gamma curve between 2 choices (0/1)
// ILI9341 only
//
int spilcdSetGamma(int iMode)
{
int i;
unsigned char *sE0, *sE1;
static unsigned char ucE0_0[] = {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00};
static unsigned char ucE1_0[] = {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F};
static unsigned char ucE0_1[] = {0x1f, 0x1a, 0x18, 0x0a, 0x0f, 0x06, 0x45, 0x87, 0x32, 0x0a, 0x07, 0x02, 0x07, 0x05, 0x00};
static unsigned char ucE1_1[] = {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3a, 0x78, 0x4d, 0x05, 0x18, 0x0d, 0x38, 0x3a, 0x1f};

	if (iMode < 0 || iMode > 1 || iLCDType != LCD_ILI9341)
		return 1;
	if (iMode == 0)
	{
		sE0 = ucE0_0;
		sE1 = ucE1_0;
	}
	else
	{
		sE0 = ucE0_1;
		sE1 = ucE1_1;
	}
	spilcdWriteCommand(0xe0);
	for(i=0; i<16; i++)
	{
		spilcdWriteData8(*sE0++);
	}
	spilcdWriteCommand(0xe1);
	for(i=0; i<16; i++)
	{
		spilcdWriteData8(*sE1++);
	}

	return 0;
} /* spilcdSetGamme() */

//
// Initialize the LCD controller and clear the display
// LED pin is optional - pass as -1 to disable
//
int spilcdInit(int iType, int bFlipped, int iChannel, int iSPIFreq, int iDC, int iReset, int iLED)
{
unsigned char *s;
int i, iCount;

	iLEDPin = -1; // assume it's not defined
	if (iType != LCD_ILI9341 && iType != LCD_ST7735 && iType != LCD_HX8357 && iType != LCD_SSD1351 && iType != LCD_ILI9342 && iType != LCD_ST7789)
	{
		printf("Unsupported display type\n");
		return -1;
	}
	iLCDType = iType;
	iScrollOffset = 0; // current hardware scroll register value

#ifdef USE_BCM2835
     iDCPin = iBCM2835Pins[iDC]; // use the pin numbers as-is
     iResetPin = iBCM2835Pins[iReset];
     if (iLED != -1)
        iLEDPin = iBCM2835Pins[iLED];
     if (iDCPin == -1 || iResetPin == -1)
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
	iDCPin = iPIGPins[iDC];
	iResetPin = iPIGPins[iReset];
        if (iLED != -1)
	   iLEDPin = iPIGPins[iLED];
	if (iDCPin == -1 || iResetPin == -1)
	{
		printf("One or more invalid GPIO pin numbers\n");
		return -1;
	}
        if (gpioInitialise() < 0)
                printf("pigpio failed to initialize\n");
        file_spi = spiOpen(iChannel, iSPIFreq, 0);
#endif // USE_PIGPIO

#ifdef USE_GENERIC
	iDCPin = iGenericPins[iDC];
	iResetPin = iGenericPins[iReset];
	if (iLED != -1)
		iLEDPin = iGenericPins[iLED];
	if (iDCPin == -1 || iResetPin == -1)
	{
		printf("One or more invalid GPIO pin numbers\n");
		return -1;
	}
	{
	char szName[32];
	int rc, iSPIMode = SPI_MODE_0; // | SPI_NO_CS;
	int i = iSPIFreq;
	sprintf(szName,"/dev/spidev%d.0", iChannel);
	file_spi = open(szName, O_RDWR);
	rc = ioctl(file_spi, SPI_IOC_WR_MODE, &iSPIMode);
	if (rc < 0) printf("Error setting SPI mode\n");
	rc = ioctl(file_spi, SPI_IOC_WR_MAX_SPEED_HZ, &i);
	if (rc < 0) printf("Error setting SPI speed\n");
	memset(&xfer, 0, sizeof(xfer));
	xfer.speed_hz = iSPIFreq;
	xfer.cs_change = 0;
	xfer.delay_usecs = 0;
	xfer.bits_per_word = 8;
//	xfer.rx_buf = (unsigned long)ucRXBuf2; // dummy receive buffer
	}
#endif // USE_GENERIC

#ifdef USE_WIRINGPI
	iDCPin = iWPPins[iDC];
	iResetPin = iWPPins[iReset];
	if (iLED != -1)
		iLEDPin = iWPPins[iLED];

	if (iDCPin == -1 || iResetPin == -1)
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

#ifdef USE_GENERIC
	GenericAddGPIO(iDCPin, GPIO_OUT, 0);
	GenericAddGPIO(iResetPin, GPIO_OUT, 0);
	if (iLEDPin != -1)
		GenericAddGPIO(iLEDPin, GPIO_OUT, 0);
#endif // USE_GENERIC

#ifdef USE_BCM2835
	bcm2835_gpio_fsel(iDCPin, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(iResetPin, BCM2835_GPIO_FSEL_OUTP);
	if (iLEDPin != -1)
        	bcm2835_gpio_fsel(iLEDPin, BCM2835_GPIO_FSEL_OUTP);
#endif
#ifdef USE_PIGPIO
	gpioSetMode(iDCPin, PI_OUTPUT);
	gpioSetMode(iResetPin, PI_OUTPUT);
	if (iLEDPin != -1)
		gpioSetMode(iLEDPin, PI_OUTPUT); 
#endif

#ifdef USE_WIRINGPI
        pinMode(iDCPin, OUTPUT);
	pinMode(iResetPin, OUTPUT);
	if (iLEDPin != -1)
		pinMode(iLEDPin, OUTPUT);
#endif

	myPinWrite(iResetPin, 1);
	usleep(100000);
	myPinWrite(iResetPin, 0); // reset the controller
	usleep(100000);
	myPinWrite(iResetPin, 1);
	usleep(200000);

	if (iLCDType != LCD_SSD1351) // no backlight and no soft reset on OLED
	{
	if (iLEDPin != -1)
		myPinWrite(iLEDPin, 1); // turn on the backlight


	spilcdWriteCommand(0x01); // software reset
	usleep(120000);

	spilcdWriteCommand(0x11);
	usleep(250000);
	}
	if (iLCDType == LCD_ST7789)
	{
		s = uc240x240InitList;
		if (bFlipped)
			s[6] = 0xc0; // flip 180
		else
			s[6] = 0x00;
		iCurrentWidth = iWidth = 240;
		iCurrentHeight = iHeight = 240;
	} // ST7789
	else if (iLCDType == LCD_SSD1351)
	{
		s = ucOLEDInitList; // do the commands manually

                iCurrentWidth = iWidth = 128;
                iCurrentHeight = iHeight = 128;
	}
    // Send the commands/parameters to initialize the LCD controller
	else if (iLCDType == LCD_ILI9341)
	{
		s = uc240InitList;
		if (bFlipped)
			s[50] = 0x88; // flip 180
		else
			s[50] = 0x48; // normal orientation
		iCurrentWidth = iWidth = 240;
		iCurrentHeight = iHeight = 320;
	}
	else if (iLCDType == LCD_ILI9342)
	{
		s = uc320InitList;
		if (bFlipped)
			s[15] = 0xc8; // flip 180
		else
			s[15] = 0x08; // normal orientation
		iCurrentWidth = iWidth = 320;
		iCurrentHeight = iHeight = 240;
	}
	else if (iLCDType == LCD_HX8357)
	{
                spilcdWriteCommand(0xb0);
                spilcdWriteData16(0x00FF);
                spilcdWriteData16(0x0001);
                usleep(100000);

		s = uc480InitList;
		if (bFlipped)
			s[65] = 0x88; // flip 180
		else
			s[65] = 0x48; // normal orientation
		iCurrentWidth = iWidth = 320;
		iCurrentHeight = iHeight = 480;
	}
	else // ST7735
	{
		s = uc128InitList;
		if (bFlipped)
			s[5] = 0x00; // flipped 180 degrees
		else
			s[5] = 0xc0; // normal orientation
		iCurrentWidth = iWidth = 128;
		iCurrentHeight = iHeight = 160;
	}
	iCount = 1;
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

	if (iLCDType != LCD_SSD1351)
	{
		spilcdWriteCommand(0x11); // sleep out
		usleep(120000);
		spilcdWriteCommand(0x29); // Display ON
		usleep(10000);
	}

	spilcdFill(0); // erase memory
	spilcdScrollReset();
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

#ifdef USE_GENERIC
	iGPIO = iGenericPins[iPin];
	if (iGPIO == -1) // invalid pin number
		return -1;
	GenericAddGPIO(iGPIO, GPIO_IN, 1);
#endif // USE_GENERIC

#ifdef USE_BCM2835
        iGPIO = iBCM2835Pins[iPin];
        if (iGPIO == 0) // invalid pin number
                return -1;
        bcm2835_gpio_fsel(iGPIO, BCM2835_GPIO_FSEL_INPT);
        bcm2835_gpio_set_pud(iGPIO,  BCM2835_GPIO_PUD_UP);
#endif
#ifdef USE_PIGPIO
        iGPIO = iPIGPins[iPin];
        if (iGPIO == -1) // invalid pin
                return -1;
        gpioSetMode(iGPIO, PI_INPUT);
        gpioSetPullUpDown(iGPIO, PI_PUD_UP);
#endif

#ifdef USE_WIRINGPI
        iGPIO = iWPPins[iPin];
        if (iGPIO == -1) // invalid
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

#ifdef USE_GENERIC
{
char szTemp[64];
int rc;

	iGPIO = iGenericPins[iPin];
	if (iGPIO == -1) // invalid pin, return idle button
		return 1;
	if (iPinHandles[iGPIO] == -1)
	{
		sprintf(szTemp, "/sys/class/gpio/gpio%d/value", iGPIO);
		iPinHandles[iGPIO] = open(szTemp, O_RDONLY);
	}
	lseek(iPinHandles[iGPIO], 0, SEEK_SET);
	rc = read(iPinHandles[iGPIO], szTemp, 1);
	if (rc < 0) // do nothing
	{}
	return (szTemp[0] == '1');
}
#endif // USE_GENERIC

#ifdef USE_PIGPIO
        iGPIO = iPIGPins[iPin];
        return gpioRead(iGPIO);
#endif // USE_PIGPIO
#ifdef USE_WIRINGPI
        iGPIO = iWPPins[iPin];
        return (digitalRead(iGPIO) == HIGH);
#endif // USE_WIRINGPI
#ifdef USE_BCM2835
        iGPIO = iBCM2835Pins[iPin];
        return (bcm2835_gpio_lev(iGPIO) == HIGH);
#endif // USE_BCM2835
} /* spilcdReadPin() */

//
// Reset the scroll position to 0
//
void spilcdScrollReset(void)
{
	iScrollOffset = 0;
	if (iLCDType == LCD_SSD1351)
	{
		spilcdWriteCommand(0xa1); // set scroll start line
		spilcdWriteData8(0x00);
		spilcdWriteCommand(0xa2); // display offset
		spilcdWriteData8(0x00);
		return;
	}
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
	if (iLCDType == LCD_SSD1351)
	{
		spilcdWriteCommand(0xa1); // set scroll start line
		spilcdWriteData8(iScrollOffset);
		return;
	}
	else
	{
		spilcdWriteCommand(0x37); // Vertical scrolling start address
		if (iLCDType == LCD_ILI9341 || iLCDType == LCD_ILI9342 || iLCDType == LCD_ST7735 || iLCDType == LCD_ST7789)
		{
			spilcdWriteData16(iScrollOffset);
		}
		else
		{
			spilcdWriteData16(iScrollOffset >> 8);
			spilcdWriteData16(iScrollOffset & -1);
		}
	}
	if (iFillColor != -1) // fill the exposed lines
	{
	int i, iStart;
	uint16_t usTemp[320];
	uint32_t *d;
	uint32_t u32Fill;
		// quickly prepare a full line's worth of the color
		u32Fill = (iFillColor >> 8) | ((iFillColor & -1) << 8);
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
		if (iOrientation == LCD_ORIENTATION_ROTATED)
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
int i, ty, th, iPerLine, iStart;

	// check bounds
	if (x < 0 || x >= iCurrentWidth || x+w > iCurrentWidth)
		return; // out of bounds
	if (y < 0 || y >= iCurrentHeight || y+h > iCurrentHeight)
		return;
	u32Color = usColor >> 8;
	u32Color |= (usColor & -1) << 8;
	u32Color |= (u32Color << 16);
	pu32 = (uint32_t *)&ucTemp[0];
	for (i=0; i<240; i++) // prepare big buffer of color
		*pu32++ = u32Color;

	ty = (iCurrentWidth == iWidth) ? y:x;
	th = (iCurrentWidth == iWidth) ? h:w;
	if (bFill)
	{
		iPerLine = (iCurrentWidth == iWidth) ? w*2:h*2; // bytes to write per line
	       	spilcdSetPosition(x, y, w, h);
	        if (((ty + iScrollOffset) % iHeight) > iHeight-th) // need to write in 2 parts since it won't wrap
		{
               		iStart = (iHeight - ((ty+iScrollOffset) % iHeight));
			for (i=0; i<iStart; i++)
                		spilcdWriteDataBlock(ucTemp, iStart*iPerLine); // first N lines
			if (iCurrentWidth == iWidth)
				spilcdSetPosition(x, y+iStart, w, h-iStart);
			else
				spilcdSetPosition(x+iStart, y, w-iStart, h);
			for (i=0; i<th-iStart; i++)
               	 		spilcdWriteDataBlock(ucTemp, iPerLine);
       		 }
        	else // can write in one shot
        	{
			for (i=0; i<th; i++)
               		 	spilcdWriteDataBlock(ucTemp, iPerLine);
        	}
	}
	else // outline
	{
		// draw top/bottom
		spilcdSetPosition(x, y, w, 1);
		spilcdWriteDataBlock(ucTemp, w*2);
		spilcdSetPosition(x, y + h-1, w, 1);
		spilcdWriteDataBlock(ucTemp, w*2);
		// draw left/right
		if (((ty + iScrollOffset) % iHeight) > iHeight-th)	
		{
			iStart = (iHeight - ((ty+iScrollOffset) % iHeight));
			spilcdSetPosition(x, y, 1, iStart);
			spilcdWriteDataBlock(ucTemp, iStart*2);
			spilcdSetPosition(x+w-1, y, 1, iStart);
			spilcdWriteDataBlock(ucTemp, iStart*2);
			// second half
			spilcdSetPosition(x,y+iStart, 1, h-iStart);
			spilcdWriteDataBlock(ucTemp, (h-iStart)*2);
			spilcdSetPosition(x+w-1, y+iStart, 1, h-iStart);
			spilcdWriteDataBlock(ucTemp, (h-iStart)*2);
		}
		else // can do it in 1 shot
		{
			spilcdSetPosition(x, y, 1, h);
			spilcdWriteDataBlock(ucTemp, h*2);
			spilcdSetPosition(x + w-1, y, 1, h);
			spilcdWriteDataBlock(ucTemp, h*2);
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
		if (iLCDType == LCD_SSD1351)
			spilcdWriteCommand(0xae); // Display Off
		else
			spilcdWriteCommand(0x29); // Display OFF
#ifdef USE_GENERIC
	close(file_spi);
	GenericRemoveGPIO(iDCPin);
	GenericRemoveGPIO(iResetPin);
	if (iLEDPin != -1)
		GenericRemoveGPIO(iLEDPin);
#endif // USE_GENERIC

#ifdef USE_PIGPIO
		spiClose(file_spi);
#endif
#ifdef USE_WIRINGPI
		close(file_spi);
#endif // USE_WIRINGPI

		file_spi = -1;
		if (iLEDPin != -1)
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

	if (iOrientation == LCD_ORIENTATION_ROTATED) // rotate 90 clockwise
	{
		// rotate the coordinate system
		t = x;
		x = iWidth-y-h;
		y = t;
		// flip the width/height too
		t = w;
		w = h;
		h = t;
	}
	y = (y + iScrollOffset) % iHeight; // scroll offset affects writing position

	if (iLCDType == LCD_SSD1351) // OLED has very different commands
	{
		spilcdWriteCommand(0x15); // set column
		ucBuf[0] = x;
		ucBuf[1] = x + w - 1;
		myspiWrite(ucBuf, 2);
		spilcdWriteCommand(0x75); // set row
		ucBuf[0] = y;
		ucBuf[1] = y + h - 1;
		myspiWrite(ucBuf, 2);
		spilcdWriteCommand(0x5c); // write RAM
		return;
	}
	spilcdWriteCommand(0x2a); // set column address
	if (iLCDType == LCD_ILI9341 || iLCDType == LCD_ILI9342 || iLCDType == LCD_ST7735 || iLCDType == LCD_ST7789)
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
	if (iLCDType == LCD_ILI9341 || iLCDType == LCD_ILI9342 || iLCDType == LCD_ST7735 || iLCDType == LCD_ST7789)
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
// Draw a 16x16 tile as 16x13 (with priority to non-black pixels)
// This is for drawing a 224x288 image onto a 320x240 display in landscape
//
int spilcdDrawRetroTile(int x, int y, unsigned char *pTile, int iPitch)
{
unsigned char ucTemp[416];
int i, j, iPitch16;
uint16_t *s, *d, u16A, u16B;

        if (file_spi < 0) return -1;

        // scale y coordinate for shrinking
        y = (y * 13)/16;
        iPitch16 = iPitch/2;
        for (j=0; j<16; j++) // 16 destination columns
        {
                s = (uint16_t *)&pTile[j * 2];
                d = (uint16_t *)&ucTemp[j*26];
                for (i=0; i<16; i++) // 13 actual source rows
                {
			if (i == 0 || i == 5 || i == 10) // combined pixels
			{
				u16A = s[(15-i)*iPitch16];
				u16B = s[(14-i)*iPitch16];
				if (u16A == 0)
					*d++ = __builtin_bswap16(u16B);
				else
					*d++ = __builtin_bswap16(u16A);
				i++; // advance count since we merged 2 lines 
			}
			else // just copy
			{
                                *d++ = __builtin_bswap16(s[(15-i)*iPitch16]);
                        }
                } // for i
        } // for j
        spilcdSetPosition(x, y, 16, 13);
        if (((x + iScrollOffset) % iHeight) > iHeight-16) // need to write in 2 parts since it won't wrap
        {
                int iStart = (iHeight - ((x+iScrollOffset) % iHeight));
                spilcdWriteDataBlock(ucTemp, iStart*26); // first N lines
                spilcdSetPosition(x+iStart, y, 16-iStart, 13);
                spilcdWriteDataBlock(&ucTemp[iStart*26], 416-(iStart*26));
        }
        else // can write in one shot
        {
                spilcdWriteDataBlock(ucTemp, 416);
        }
        return 0;

} /* spilcdDrawRetroTile() */

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
        spilcdSetPosition(x, y, 16, 14);
        if (((x + iScrollOffset) % iHeight) > iHeight-16) // need to write in 2 parts since it won't wrap
        {
                int iStart = (iHeight - ((x+iScrollOffset) % iHeight));
                spilcdWriteDataBlock(ucTemp, iStart*28); // first N lines
                spilcdSetPosition(x+iStart, y, 16-iStart, 14);
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
// The x/y coordinates will be scaled 2x in the X direction and 1.5x in the Y
// It is assumed that the display is set to ROTATED orientation
//
int spilcdDrawScaledTile(int x, int y, int cx, int cy, unsigned char *pTile, int iPitch)
{
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
	for (j=0; j<cx; j+=2) // source lines (2 at a time)
	{
		s = (uint32_t *)&pTile[j * 2];
		d = (uint16_t *)&ucRXBuf[j*cy*6];
		for (i=0; i<cy; i+=2) // source columns (2 at a time)
		{
			u32A = s[(cy-1-i)*iPitch32];
			u32B = s[(cy-2-i)*iPitch32];
			u32C = u32A >> 16;
			u32D = u32B >> 16;
			u32A &= u32Mask;
			u32B &= u32Mask;
			u32a = (u32A & u32Magic) >> 1;
			u32a += ((u32B & u32Magic) >> 1);
			u32b = (u32C & u32Magic) >> 1;
			u32b += ((u32D & u32Magic) >> 1);
			d[0] = d[(cy*3)/2] = __builtin_bswap16(u32A); // swap byte order
			d[1] = d[((cy*3)/2)+1] = __builtin_bswap16(u32a);
			d[2] = d[((cy*3)/2)+2] = __builtin_bswap16(u32B);
			d[cy*3] = d[(cy*9)/2] = __builtin_bswap16(u32C);
			d[(cy*3)+1] = d[((cy*9)/2)+1] = __builtin_bswap16(u32b);
			d[(cy*3)+2] = d[((cy*9)/2)+2] = __builtin_bswap16(u32D);
			d += 3;
		} // for i
	} // for j
        spilcdSetPosition(x, y, cx*2, (cy*3)/2);
        if (((x + iScrollOffset) % iHeight) > iHeight-(cx*2)) // need to write in 2 parts since it won't wrap
        {
                int iStart = (iHeight - ((x+iScrollOffset) % iHeight));
                spilcdWriteDataBlock(ucRXBuf, iStart*cx*3); // first N lines
                spilcdSetPosition(x+iStart, y, (cx*2)-iStart, (cy*3)/2);
                spilcdWriteDataBlock(&ucRXBuf[iStart*cx*3], (cx*cy*6)-(iStart*cx*3));
        }
        else // can write in one shot
        {
                spilcdWriteDataBlock(ucRXBuf, (cx*cy*6));
        }
	return 0;
} /* spilcdDrawScaledTile() */
//
// Draw a 24x24 RGB565 tile scaled to 40x40
// The main purpose of this function is for GameBoy emulation
// Since the original display is 160x144, this function allows it to be 
// stretched 166% larger (266x240). Not a perfect fit for 320x240, but better
// Each group of 3x3 pixels becomes a group of 5x5 pixels by averaging the pixels
//
// +-+-+-+ becomes +----+----+----+----+----+
// |A|B|C|         |A   |ab  |B   |bc  |C   |
// +-+-+-+         +----+----+----+----+----+
// |D|E|F|         |ad  |abde|be  |becf|cf  |
// +-+-+-+         +----+----+----+----+----+
// |G|H|I|         |D   |de  |E   |ef  |F   |
// +-+-+-+         +----+----+----+----+----+
//                 |dg  |dgeh|eh  |ehfi|fi  |
//                 +----+----+----+----+----+
//                 |G   |gh  |H   |hi  |I   |
//                 +----+----+----+----+----+
//
// The x/y coordinates will be scaled as well
//
int spilcdDraw53Tile(int x, int y, int cx, int cy, unsigned char *pTile, int iPitch)
{
int i, j, iPitch16;
uint16_t *s, *d;
uint32_t u32A, u32B, u32C, u32D, u32E, u32F;
uint32_t t1, t2, u32ab, u32bc, u32de, u32ef, u32ad, u32be, u32cf;
uint32_t u32Magic = 0xf7def7de;
int bFlipped;

	if (file_spi < 0) return -1;
	bFlipped = (iWidth < 320); // rotated display

	// scale coordinates for stretching
	x = (x * 5)/3;
	y = (y * 5)/3;
        iPitch16 = iPitch/2;
	if (cx < 24 || cy < 24)
		memset(ucRXBuf, 0, 40*40*2);
	for (j=0; j<cy/3; j++) // 8 blocks of 3 lines
	{
		s = (uint16_t *)&pTile[j*3*iPitch];
		if (bFlipped)
			d = (uint16_t *)&ucRXBuf[(35-(j*5))*2];
		else
			d = (uint16_t *)&ucRXBuf[j*40*5*2];
		for (i=0; i<cx-2; i+=3) // source columns (3 at a time)
		{
			u32A = s[i];
			u32B = s[i+1];
			u32C = s[i+2];
			u32D = s[i+iPitch16];
			u32E = s[i+iPitch16+1];
			u32F = s[i+iPitch16 + 2];
			u32bc = u32ab = (u32B & u32Magic) >> 1;
			u32ab += ((u32A & u32Magic) >> 1);
			u32bc += (u32C & u32Magic) >> 1;
			u32de = u32ef = ((u32E & u32Magic) >> 1);
			u32de += ((u32D & u32Magic) >> 1);
			u32ef += ((u32F & u32Magic) >> 1);
			u32ad = ((u32A & u32Magic) >> 1) + ((u32D & u32Magic) >> 1);
			u32be = ((u32B & u32Magic) >> 1) + ((u32E & u32Magic) >> 1);
			u32cf = ((u32C & u32Magic) >> 1) + ((u32F & u32Magic) >> 1);
			// first row
			if (bFlipped)
			{
			d[4] = __builtin_bswap16(u32A); // swap byte order
			d[44] = __builtin_bswap16(u32ab);
			d[84] = __builtin_bswap16(u32B);
			d[124] = __builtin_bswap16(u32bc);
			d[164] = __builtin_bswap16(u32C);
			}
			else
			{
			d[0] = __builtin_bswap16(u32A); // swap byte order
			d[1] = __builtin_bswap16(u32ab);
			d[2] = __builtin_bswap16(u32B);
			d[3] = __builtin_bswap16(u32bc);
			d[4] = __builtin_bswap16(u32C);
			}
			// second row
			t1 = ((u32ab & u32Magic) >> 1) + ((u32de & u32Magic) >> 1);
			t2 = ((u32be & u32Magic) >> 1) + ((u32cf & u32Magic) >> 1);
			if (bFlipped)
			{
			d[3] = __builtin_bswap16(u32ad);
			d[43] = __builtin_bswap16(t1);
			d[83] = __builtin_bswap16(u32be);
			d[123] = __builtin_bswap16(t2);
			d[163] = __builtin_bswap16(u32cf);
			}
			else
			{
			d[40] = __builtin_bswap16(u32ad);
			d[41] = __builtin_bswap16(t1);
			d[42] = __builtin_bswap16(u32be);
			d[43] = __builtin_bswap16(t2);
			d[44] = __builtin_bswap16(u32cf);
			}
			// third row
			if (bFlipped)
			{
			d[2] = __builtin_bswap16(u32D);
			d[42] = __builtin_bswap16(u32de);
			d[82] = __builtin_bswap16(u32E);
			d[122] = __builtin_bswap16(u32ef);
			d[162] = __builtin_bswap16(u32F);
			}
			else
			{
			d[80] = __builtin_bswap16(u32D);
			d[81] = __builtin_bswap16(u32de);
			d[82] = __builtin_bswap16(u32E);
			d[83] = __builtin_bswap16(u32ef);
			d[84] = __builtin_bswap16(u32F);
			}
			// fourth row
			u32A = s[i+iPitch16*2];
			u32B = s[i+iPitch16*2 + 1];
			u32C = s[i+iPitch16*2 + 2];
			u32bc = u32ab = (u32B & u32Magic) >> 1;
			u32ab += ((u32A & u32Magic) >> 1);
			u32bc += (u32C & u32Magic) >> 1;
			u32ad = ((u32A & u32Magic) >> 1) + ((u32D & u32Magic) >> 1);
			u32be = ((u32B & u32Magic) >> 1) + ((u32E & u32Magic) >> 1);
			u32cf = ((u32C & u32Magic) >> 1) + ((u32F & u32Magic) >> 1);
			t1 = ((u32ab & u32Magic) >> 1) + ((u32de & u32Magic) >> 1);
			t2 = ((u32be & u32Magic) >> 1) + ((u32cf & u32Magic) >> 1);
			if (bFlipped)
			{
			d[1] = __builtin_bswap16(u32ad);
			d[41] = __builtin_bswap16(t1);
			d[81] = __builtin_bswap16(u32be);
			d[121] = __builtin_bswap16(t2);
			d[161] = __builtin_bswap16(u32cf);
			}
			else
			{
			d[120] = __builtin_bswap16(u32ad);
			d[121] = __builtin_bswap16(t1);
			d[122] = __builtin_bswap16(u32be);
			d[123] = __builtin_bswap16(t2);
			d[124] = __builtin_bswap16(u32cf);
			}
			// fifth row
			if (bFlipped)
			{
			d[0] = __builtin_bswap16(u32A);
			d[40] = __builtin_bswap16(u32ab);
			d[80] = __builtin_bswap16(u32B);
			d[120] = __builtin_bswap16(u32bc);
			d[160] = __builtin_bswap16(u32C);
			d += 200;
			}
			else
			{
			d[160] = __builtin_bswap16(u32A);
			d[161] = __builtin_bswap16(u32ab);
			d[162] = __builtin_bswap16(u32B);
			d[163] = __builtin_bswap16(u32bc);
			d[164] = __builtin_bswap16(u32C);
			d += 5;
			}
		} // for i
	} // for j
        spilcdSetPosition(x, y, 40, 40);
        spilcdWriteDataBlock(ucRXBuf, 40*40*2);
	return 0;
} /* spilcdDraw53Tile() */
//
// Draw a 16x16 RGB656 tile with select rows/columns removed
// the mask contains 1 bit for every column/row that should be drawn
// For example, to skip the first 2 columns, the mask value would be 0xfffc
//
int spilcdDrawMaskedTile(int x, int y, unsigned char *pTile, int iPitch, int iColMask, int iRowMask)
{
unsigned char ucTemp[512]; // fix the byte order first to write it more quickly
int i, j;
unsigned char *s, *d;
int iNumCols, iNumRows, iTotalSize;

	iNumCols = __builtin_popcount(iColMask);
	iNumRows = __builtin_popcount(iRowMask);
	iTotalSize = iNumCols * iNumRows * 2;

        if (file_spi < 0) return -1;

        if (iOrientation == LCD_ORIENTATION_ROTATED) // need to rotate the data
        {
                // First convert to big-endian order
                d = ucTemp;
                for (j=0; j<16; j++)
                {
			if ((iColMask & (1<<j)) == 0) continue; // skip row
                        s = &pTile[j*2];
                        for (i=0; i<16; i++)
                        {
				if ((iRowMask & (1<<i)) == 0) continue; // skip column
                                d[1] = s[(15-i)*iPitch];
                                d[0] = s[((15-i)*iPitch)+1]; // swap byte order (MSB first)
                                d += 2;
                        } // for i;
                } // for j
                spilcdSetPosition(x, y, iNumCols, iNumRows);
                if (((x + iScrollOffset) % iHeight) > iHeight-iNumRows) // need to write in 2 parts since it won't wrap
                {
                        int iStart = (iHeight - ((x+iScrollOffset) % iHeight));
                        spilcdWriteDataBlock(ucTemp, iStart*iNumRows*2); // first N lines
                        spilcdSetPosition(x+iStart, y, iNumRows-iStart, iNumCols);
                        spilcdWriteDataBlock(&ucTemp[iStart*iNumRows*2], iTotalSize-(iStart*iNumRows*2));
                }
                else // can write in one shot
                {
                        spilcdWriteDataBlock(ucTemp, iTotalSize);
                }
        }
        else // native orientation
        {
	uint16_t *s16 = (uint16_t *)s;
	uint16_t u16, *d16 = (uint16_t *)d;
	int iMask;

        // First convert to big-endian order
        d16 = (uint16_t *)ucTemp;
        for (j=0; j<16; j++)
        {
		if ((iRowMask & (1<<j)) == 0) continue; // skip row
                s16 = (uint16_t *)&pTile[j*iPitch];
		iMask = iColMask;
                for (i=0; i<16; i++)
                {
			u16 = *s16++;
			if (iMask & 1)
			{
                        	*d16++ = __builtin_bswap16(u16);
                        }
			iMask >>= 1;
                } // for i;
        } // for j
        spilcdSetPosition(x, y, iNumCols, iNumRows);
        if (((y + iScrollOffset) % iHeight) > iHeight-iNumRows) // need to write in 2 parts since it won't wrap
        {
                int iStart = (iHeight - ((y+iScrollOffset) % iHeight));
                spilcdWriteDataBlock(ucTemp, iStart*iNumCols*2); // first N lines
                spilcdSetPosition(x, y+iStart, iNumCols, iNumRows-iStart);
                spilcdWriteDataBlock(&ucTemp[iStart*iNumCols*2], iTotalSize-(iStart*iNumCols*2));
        }
        else // can write in one shot
        {
                spilcdWriteDataBlock(ucTemp, iTotalSize);
        }
        } // portrait orientation
        return 0;
} /* spilcdDrawMaskedTile() */

//
// Draw a NxN RGB565 tile
// This reverses the pixel byte order and sets a memory "window"
// of pixels so that the write can occur in one shot
//
int spilcdDrawTile(int x, int y, int iTileWidth, int iTileHeight, unsigned char *pTile, int iPitch)
{
int i, j;
uint32_t ul32;
unsigned char *s, *d;

	if (file_spi < 0) return -1;
	if (iTileWidth*iTileHeight > 2048)
		return -1; // tile must fit in 4k SPI block size

	if (iOrientation == LCD_ORIENTATION_ROTATED) // need to rotate the data
	{
        	// First convert to big-endian order
        	d = ucRXBuf;
        	for (j=0; j<iTileWidth; j++)
        	{
                	s = &pTile[j*2];
			s += (iTileHeight-2)*iPitch; 
                	for (i=0; i<iTileHeight; i+=2)
                	{
				// combine the 2 pixels into a single write for better memory performance
                        	ul32 = __builtin_bswap16(*(uint16_t *)&s[iPitch]);
                        	ul32 |= (__builtin_bswap16(*(uint16_t *)s) << 16); // swap byte order (MSB first)
				*(uint32_t *)d = ul32;
                        	d += 4;
				s -= iPitch*2;
                	} // for i;
        	} // for j
        	spilcdSetPosition(x, y, iTileWidth, iTileHeight);
		if (((x + iScrollOffset) % iHeight) > iHeight-iTileWidth) // need to write in 2 parts since it won't wrap
		{
			int iStart = (iHeight - ((x+iScrollOffset) % iHeight));
			spilcdWriteDataBlock(ucRXBuf, iStart*iTileHeight*2); // first N lines
			spilcdSetPosition(x+iStart, y, iTileWidth-iStart, iTileWidth); 
			spilcdWriteDataBlock(&ucRXBuf[iStart*iTileHeight*2], (iTileWidth*iTileHeight*2)-(iStart*iTileHeight*2));
		}
		else // can write in one shot
		{
			spilcdWriteDataBlock(ucRXBuf, iTileWidth*iTileHeight*2);
		}
	}
	else // native orientation
	{
        uint16_t *s16, *d16;
	// First convert to big-endian order
	d16 = (uint16_t *)ucRXBuf;
	for (j=0; j<iTileHeight; j++)
	{
		s16 = (uint16_t*)&pTile[j*iPitch];
		for (i=0; i<iTileWidth; i++)
		{
			*d16++ = __builtin_bswap16(*s16++);
		} // for i;
	} // for j
	spilcdSetPosition(x, y, iTileWidth, iTileHeight);
	if (((y + iScrollOffset) % iHeight) > iHeight-iTileHeight) // need to write in 2 parts since it won't wrap
        {
                int iStart = (iHeight - ((y+iScrollOffset) % iHeight));
                spilcdWriteDataBlock(ucRXBuf, iStart*iTileWidth*2); // first N lines
                spilcdSetPosition(x, y+iStart, iTileWidth, iTileHeight-iStart);
                spilcdWriteDataBlock(&ucRXBuf[iStart*iTileWidth*2], (iTileWidth*iTileHeight*2)-(iStart*iTileWidth*2));
        }
        else // can write in one shot
	{
        	spilcdWriteDataBlock(ucRXBuf, iTileWidth*iTileHeight*2);
	}
	} // portrait orientation
	return 0;
} /* spilcdDrawTile() */
//
// Draw a NxN RGB565 tile
// This reverses the pixel byte order and sets a memory "window"
// of pixels so that the write can occur in one shot
// Scales the tile by 150% (for GameBoy/GameGear)
//
int spilcdDrawTile150(int x, int y, int iTileWidth, int iTileHeight, unsigned char *pTile, int iPitch)
{
int i, j, iPitch32, iLocalPitch;
uint32_t ul32A, ul32B, ul32Avg, ul32Avg2;
uint16_t u16Avg, u16Avg2;
uint32_t u32Magic = 0xf7def7de;
uint16_t u16Magic = 0xf7de;
uint16_t *d16;
uint32_t *s32;

	if (file_spi < 0) return -1;
	if (iTileWidth*iTileHeight > 1365)
		return -1; // tile must fit in 4k SPI block size

	iPitch32 = iPitch / 4;
	iLocalPitch = (iTileWidth * 3)/2; // offset to next output line
	d16 = (uint16_t *)ucRXBuf;
	for (j=0; j<iTileHeight; j+=2)
	{
		s32 = (uint32_t*)&pTile[j*iPitch];
		for (i=0; i<iTileWidth; i+=2) // turn 2x2 pixels into 3x3 
		{
			ul32A = s32[0];
			ul32B = s32[iPitch32]; // get 2x2 pixels
			// top row
			ul32Avg = ((ul32A & u32Magic) >> 1);
			ul32Avg2 = ((ul32B & u32Magic) >> 1);
			u16Avg = (uint16_t)(ul32Avg + (ul32Avg >> 16)); // average the 2 pixels
			d16[0] = __builtin_bswap16((uint16_t)ul32A); // first pixel
			d16[1] = __builtin_bswap16(u16Avg); // middle (new) pixel
			d16[2] = __builtin_bswap16((uint16_t)(ul32A >> 16)); // 3rd pixel
			u16Avg2 = (uint16_t)(ul32Avg2 + (ul32Avg2 >> 16)); // bottom line averaged pixel
			d16[iLocalPitch] = __builtin_bswap16((uint16_t)(ul32Avg + ul32Avg2)); // vertical average
			d16[iLocalPitch+2] = __builtin_bswap16((uint16_t)((ul32Avg + ul32Avg2)>>16)); // vertical average
			d16[iLocalPitch*2] = __builtin_bswap16((uint16_t)ul32B); // last line 1st
			d16[iLocalPitch*2+1] = __builtin_bswap16(u16Avg2); // middle pixel
			d16[iLocalPitch*2+2] = __builtin_bswap16((uint16_t)(ul32B >> 16)); // 3rd pixel
			u16Avg = (u16Avg & u16Magic) >> 1;
			u16Avg2 = (u16Avg2 & u16Magic) >> 1;
			d16[iLocalPitch+1] = __builtin_bswap16(u16Avg + u16Avg2); // middle pixel
			d16 += 3;
			s32 += 1;
		} // for i;
		d16 += iLocalPitch*2; // skip lines we already output
	} // for j
	spilcdSetPosition((x*3)/2, (y*3)/2, (iTileWidth*3)/2, (iTileHeight*3)/2);
      	spilcdWriteDataBlock(ucRXBuf, (iTileWidth*iTileHeight*9)/2);
	return 0;
} /* spilcdDrawTile150() */

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
// Draw a string of small (8x8) text as quickly as possible
// by writing it to the LCD in a single SPI write
// The string must be 32 characters or shorter
//
int spilcdWriteStringFast(int x, int y, char *szMsg, unsigned short usFGColor, unsigned short usBGColor)
{
int i, j, k, iMaxLen, iLen;
int iChars, iStride;
unsigned char *s;
unsigned short usFG = (usFGColor >> 8) | ((usFGColor & -1)<< 8);
unsigned short usBG = (usBGColor >> 8) | ((usBGColor & -1)<< 8);
unsigned short *usD;


        if (file_spi < 0) return -1; // not initialized

        iLen = strlen(szMsg);
	if (iLen <=0) return -1; // can't use this function
        iMaxLen = (iOrientation == LCD_ORIENTATION_NATIVE) ? iWidth : iHeight;

                if ((8*iLen) + x > iMaxLen) iLen = (iMaxLen - x)/8; // can't display it all
		if (iOrientation == LCD_ORIENTATION_ROTATED) // draw rotated
                {
			iChars = 0;
			for (i=0; i<iLen; i++)
                        {
				s = &ucFont[(unsigned char)szMsg[i] * 8];
				usD = (unsigned short *)&ucRXBuf[iChars*128];
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
				iChars++;
				if (iChars == 32) // need to write it
				{
					spilcdSetPosition(x, y, 8*iChars, 8);
					spilcdWriteDataBlock(ucRXBuf, iChars*128);
					x += iChars*8;
					iChars = 0;
				}
                        } // for i
			if (iChars)
			{
				spilcdSetPosition(x, y, 8*iChars, 8);
                                spilcdWriteDataBlock(ucRXBuf, iChars*128);
			}
		} // landscape
                else // portrait orientation
                {
			if (iLen > 32) iLen = 32;
			iStride = iLen * 16;
			for (i=0; i<iLen; i++)
			{
				s = &ucFont[(unsigned char)szMsg[i] * 8];
                                for (k=0; k<8; k++) // for each scanline
                                {
					usD = (unsigned short *)&ucRXBuf[(k*iStride) + (i * 16)];
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
		spilcdSetPosition(x, y, 8*iLen, 8);
                spilcdWriteDataBlock(ucRXBuf, iLen*128);
                } // portrait orientation
	return 0;
} /* spilcdWriteStringFast() */
//
// Draw a string of small (8x8) or large (16x32) characters
// At the given col+row
//
int spilcdWriteString(int x, int y, char *szMsg, unsigned short usFGColor, unsigned short usBGColor, int bLarge)
{
int i, j, k, iMaxLen, iLen;
unsigned char *s;
unsigned short usFG = (usFGColor >> 8) | ((usFGColor & -1)<< 8);
unsigned short usBG = (usBGColor >> 8) | ((usBGColor & -1)<< 8);


	if (file_spi < 0) return -1; // not initialized

	iLen = strlen(szMsg);
	iMaxLen = (iOrientation == LCD_ORIENTATION_NATIVE) ? iWidth : iHeight;

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
			if (iOrientation == LCD_ORIENTATION_ROTATED) // rotated
			{
				spilcdSetPosition(x+(i*16), y,16,32);
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
			if (iOrientation == LCD_ORIENTATION_ROTATED) // draw rotated
			{
				spilcdSetPosition(x+(i*8), y, 8, 8);
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
	if (iOrient != LCD_ORIENTATION_NATIVE && iOrient != LCD_ORIENTATION_ROTATED)
		return -1;
	iOrientation = iOrient; // nothing else needed to do
	iCurrentWidth = (iOrientation == LCD_ORIENTATION_NATIVE) ? iWidth : iHeight;
	iCurrentHeight = (iOrientation == LCD_ORIENTATION_NATIVE) ? iHeight : iWidth;
	return 0;
} /* spilcdSetOrientation() */

//
// Fill the frame buffer with a single color
//
int spilcdFill(unsigned short usData)
{
int y;
uint32_t usC, temp[1024];
int iOldOrient;

	if (file_spi < 0) return -1; // not initialized

	usC = (usData >> 8) | ((usData & -1)<<8); // swap endian-ness
	usC |= (usC << 16);
	// make sure we're in landscape mode to use the correct coordinates
	iOldOrient = iOrientation;
	iOrientation = LCD_ORIENTATION_NATIVE;
	spilcdScrollReset();
	spilcdSetPosition(0,0,iWidth,iHeight);
	iOrientation = iOldOrient;

	for (y=0; y<iWidth*2; y++)
           temp[y] = usC;
	for (y=0; y<iHeight/4; y++)
	{
		spilcdWriteDataBlock((unsigned char *)temp, iWidth*4*2); // fill with data byte
	} // for y
	return 0;
} /* spilcdFill() */
