//==============================================================================
//
//  Crystalfontz America Inc.
//
//  The controller is a Sitronix ST7789V
//    https://www.crystalfontz.com/controllers/Sitronix/ST7789V/
//
//  Seeeduino, an open-source 3.3v capable Arduino clone.
//    https://www.crystalfontz.com/product/cfapn15062-seeeduino-arduino-clone-microprocessor
//    https://github.com/SeeedDocument/SeeeduinoV4/raw/master/resources/Seeeduino_v4.2_sch.pdf
//
//==============================================================================
// This is free and unencumbered software released into the public domain.
//
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.
//
// In jurisdictions that recognize copyright laws, the author or authors
// of this software dedicate any and all copyright interest in the
// software to the public domain. We make this dedication for the benefit
// of the public at large and to the detriment of our heirs and
// successors. We intend this dedication to be an overt act of
// relinquishment in perpetuity of all present and future rights to this
// software under copyright law.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
// OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// For more information, please refer to <http://unlicense.org/>
//
// Connection guide:
//==============================================================================
// LCD SPI pins and control lines on Seeeduino:
//  ARD      | Port  | Display pin |  Function - SPI                          |
//-----------+-------+-------------+------------------------------------------+
// 3.3V/5V   |       |             |  POWER 3.3V                              |
// GND       |       |             |  GROUND                                  |
//-----------+-------+-------------+------------------------------------------+
// D8        | PORTB |  32         |  Data/Command                    (DC)    |
// D9        | PORTB |  35         |  Reset                           (Reset) |
// D10       | PORTB |  34         |  Chip select                     (CS)    |
// D11       | PORTB |  24         |  SPI data input                  (SDA)   |
// D13       | PORTB |  33         |  Serial clock                    (SCK)   |
//-----------+-------+-------------+------------------------------------------+
//==============================================================================
// Interface Selection
// IM2 | IM1 | IM0 |  Interface mode  |
//-----+-----+-----+------------------+
// 0   | 0   | 1   |  8-bit parallel  |
// 1   | 0   | 1   |  3-wire SPI      |
// 1   | 1   | 0   |  4-wire SPI      |
//-----+-----+-----+------------------+
//==============================================================================
// Resistive touchscreen connection (SPI only)
//  ARD      | Port  | Touchscreen pin |  Function                            |
//-----------+-------+-----------------+---------------- ---------------------+
// D14/A0    | PORTC |  XL (43)        |  Touch panel left            (XL)    |
// D15/A1    | PORTC |  XR (45)        |  Touch panel right           (XR)    |
// D16/A2    | PORTC |  YD (44)        |  Touch panel down            (YD)    |
// D17/A3    | PORTC |  YU (42)        |  Touch panel up              (YU)    |
//-----------+-------+-----------------+---------------- ---------------------+
//==============================================================================
// SD card connection (using CFA10112)
//  ARD      | Port  | Adapter pin |  Function                                |
//-----------+-------+-------------+------------------------------------------+
// 3.3V      |       |  3.3V       |  POWER 3.3V                              |
// GND       |       |  GND        |  GROUND                                  |
//-----------+-------+-------------+------------------------------------------+
// D7        | PORTC |  CS         |  Chip select                     (CS)    |
// D11       | PORTB |  DI         |  Serial data in                  (DI)    |
// D12       | PORTB |  DO         |  Serial data out                 (DO)    |
// D13       | PORTB |  SCK        |  Serial clock                    (SCK)   |
//-----------+-------+-------------+------------------------------------------+

//==============================================================================
// all includes are here
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// setup display/arduino related defines 
#define touch_display 1 // options are RESISTIVE (1) or NONE (0)

#define DC   (8)
#define RES  (9)
#define CS   (10)
#define MOSI (11)
#define SCLK (13)

#define DC_MASK   (0x01)
#define CLR_DC    (PORTB &= ~(DC_MASK))
#define SET_DC    (PORTB |= (DC_MASK))
#define RES_MASK  (0x02)
#define CLR_RES   (PORTB &= ~(RES_MASK))
#define SET_RES   (PORTB |= (RES_MASK))
#define CS_MASK   (0x04)
#define CLR_CS    (PORTB &= ~(CS_MASK))
#define SET_CS    (PORTB |= (CS_MASK))
#define MOSI_MASK (0x08)
#define CLR_MOSI  (PORTB &= ~(MOSI_MASK))
#define SET_MOSI  (PORTB |= (MOSI_MASK))
#define SCLK_MASK (0x20)
#define CLR_SCLK  (PORTB &= ~(SCLK_MASK))
#define SET_SCLK  (PORTB |= (SCLK_MASK))

// setup display controller related defines
#define ST7789_11_SLPOUT (0x11) 
#define ST7789_21_INVON (0x21) 
#define ST7789_29_DISPON (0x29)
#define ST7789_2A_CASET (0x2A) 
#define ST7789_2B_RASET (0x2B) 
#define ST7789_2C_RAMWR (0x2C)
#define ST7789_36_MADCTL (0x36) 
#define ST7789_3A_COLMOD (0x3A) 
#define ST7789_B2_PORCTRL (0xB2)
#define ST7789_B7_GCTRL (0xB7) 
#define ST7789_BB_VCOMS (0xBB) 
#define ST7789_C0_LCMCTRL (0xC0) 
#define ST7789_C2_VDVVRHEN (0xC2) 
#define ST7789_C3_VRHS (0xC3) 
#define ST7789_C4_VDVSET (0xC4) 
#define ST7789_C6_FRCTR2 (0xC6) 
#define ST7789_D0_PWCTRL1 (0xD0) 
#define ST7789_E0_PVGAMCTRL (0xE0)
#define ST7789_E1_NVGAMCTRL (0xE1)

//==============================================================================
// LCD initialization sequence 
void init_LCD() {
	CLR_RES; // reset the display
	delay(1); 
	SET_RES;
	delay(150); // 120ms min

	SPI_sendCommand(ST7789_11_SLPOUT); // turn off sleep mode
	delay(120); 

	SPI_sendCommand(ST7789_36_MADCTL); // memory data access control
	SPI_sendData(0x08 | 0x80); // BGR + top to bottom page address order

	SPI_sendCommand(ST7789_3A_COLMOD); // interface pixel format
	SPI_sendData(0x06); // 18 bits/pixel

	SPI_sendCommand(ST7789_B2_PORCTRL); // porch control sequence
	SPI_sendData(0x0C); 
	SPI_sendData(0x0C); 
	SPI_sendData(0x00); 
	SPI_sendData(0x33); 
	SPI_sendData(0x33); 

	SPI_sendCommand(ST7789_B7_GCTRL); // gate control
	SPI_sendData(0x35);

	SPI_sendCommand(ST7789_BB_VCOMS); // VCOMS setting
	SPI_sendData(0x2B);

	SPI_sendCommand(ST7789_C0_LCMCTRL); // LCM control
	SPI_sendData(0x2C); 

	SPI_sendCommand(ST7789_C2_VDVVRHEN); // VDV and VRH command enable
	SPI_sendData(0x01); 
	SPI_sendData(0xFF);

	SPI_sendCommand(ST7789_C3_VRHS); // VRH set
	SPI_sendData(0x20);
	SPI_sendCommand(ST7789_C4_VDVSET); // VDV set
	SPI_sendData(0x20);

	SPI_sendCommand(ST7789_C6_FRCTR2); // normal mode frame rate control
	SPI_sendData(0x0F);

	SPI_sendCommand(ST7789_D0_PWCTRL1); // power control 1
	SPI_sendData(0xA4); 
	SPI_sendData(0xA1); 

	SPI_sendCommand(ST7789_E0_PVGAMCTRL); // positive gamma control
	SPI_sendData(0xD0);
	SPI_sendData(0xCA);
	SPI_sendData(0x0E);
	SPI_sendData(0x08);
	SPI_sendData(0x09);
	SPI_sendData(0x07);
	SPI_sendData(0x2D);
	SPI_sendData(0x3B);
	SPI_sendData(0x3D);
	SPI_sendData(0x34);
	SPI_sendData(0x0A);
	SPI_sendData(0x0A);
	SPI_sendData(0x1B);
	SPI_sendData(0x28);

	SPI_sendCommand(ST7789_E1_NVGAMCTRL); // negative gamma control
	SPI_sendData(0xD0);
	SPI_sendData(0xCA);
	SPI_sendData(0x0F);
	SPI_sendData(0x08);
	SPI_sendData(0x08);
	SPI_sendData(0x07);
	SPI_sendData(0x2E);
	SPI_sendData(0x5C);
	SPI_sendData(0x40);
	SPI_sendData(0x34);
	SPI_sendData(0x09);
	SPI_sendData(0x0B);
	SPI_sendData(0x1B);
	SPI_sendData(0x28);

	SPI_sendCommand(ST7789_21_INVON); // display inversion on

	SPI_sendCommand(ST7789_2A_CASET); // column address set
	SPI_sendData(0x00); // start MSB start = 0
	SPI_sendData(0x00); // start LSB
	SPI_sendData(0x00); // end MSB end  = 249
	SPI_sendData(0xEF); // end LSB

	SPI_sendCommand(ST7789_2B_RASET); // row address set
	SPI_sendData(0x00); // start MSB start = 0
	SPI_sendData(0x00); // start LSB
	SPI_sendData(0x01); // end MSB end = 319
	SPI_sendData(0x3F); // end LSB

	SPI_sendCommand(ST7789_29_DISPON); // display on
	delay(1);
}

//==============================================================================
// SPI send command/data functions are here
void SPI_sendCommand(uint8_t command) {
    // select the LCD controller
    CLR_CS;
    // select the command register 
    CLR_DC;
    // send the command out
    SPI.transfer(command);
    // de-select the LCD controller
    SET_CS;
}

void SPI_sendData(uint8_t data) {
    // select the LCD controller
    CLR_CS;
    // select the data register
    SET_DC;
    // send the data out
    SPI.transfer(data);
    // de-select the LCD controller
    SET_CS;
}

//==============================================================================
// setup and loop functions are here
void setup() {
Serial.begin(115200); // debug console

// set up port directions and initial starting states
DDRB = 0x3F; // pins 8-13 as outputs
DDRC = 0x0F; 
PORTB = 0x00;
PORTC = 0x00;

CLR_RES;
CLR_DC;
SET_CS;
CLR_MOSI;
CLR_SCLK;

// setup SD CS pins and SPI parameters
SD.begin(7);
SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
SPI.begin();

init_LCD(); // call the display initialization function
}

// set the demo to 1 to try it out. more demos will be added!
#define res_touch_demo 1 // touch the top left corner to exit the demo
#define find_minandmax 0 // for simple touch calibration (if existing values do not work well)
#define showcolor_demo 0
#define colorbars_demo 0
#define show_BMPs_demo 1 // BMP images must be saved in the root directory of the SD card
						 // they must be exactly 240x320 pixels in size and in 24-bit color

void loop() {
#if showcolor_demo
    // colors are printed in the B-G-R format
	fill_LCD(0x00, 0xFF, 0x00);
	delay(1000);
	fill_LCD(0xFF, 0x00, 0x00);
	delay(1000);
	fill_LCD(0x00, 0x00, 0xFF);
	delay(1000);
	fill_LCD(0xFF, 0xFF, 0xFF);
	delay(1000);
	fill_LCD(0x00, 0x00, 0x00);
	delay(1000);
#endif

#if colorbars_demo
	ColorBars();
	delay(5000);
#endif

#if show_BMPs_demo
	show_BMPs_in_root();
	delay(5000);
#endif

#if res_touch_demo
	fill_LCD(0x00, 0x0F, 0xA6);
	touchDemo();
#endif

#if find_minandmax
  // simple touch screen calibration
  // enable this loop to find limits of the touch screen
  // stroke with a stylus from center to outside in every
  // direction
  // record the values from the serial monitor into the code below
  while (1)
  {
    uint16_t X;
    uint16_t Y;

    // read X. set a gradient from 0v to 3.3V on X, then
    // read the Y line to get the X contact point
    pinMode(TS_YU, INPUT);
    pinMode(TS_YD, INPUT);
    pinMode(TS_XL, OUTPUT);
    digitalWrite(TS_XL, HIGH);
    pinMode(TS_XR, OUTPUT);
    digitalWrite(TS_XR, LOW);
    X = analogRead(TS_YU);

    // read Y. Set a gradient from 0v to 3.3V on Y, then
    // read the X line to get the Y contact point
    pinMode(TS_XL, INPUT);
    pinMode(TS_XR, INPUT);
    pinMode(TS_YU, OUTPUT);
    digitalWrite(TS_YU, HIGH);
    pinMode(TS_YD, OUTPUT);
    digitalWrite(TS_YD, LOW);
    Y = analogRead(TS_XL);

    if (X < Xmin)
    {
      Xmin = X;
    }
    if (Xmax < X)
    {
      Xmax = X;
    }
    if (Y < Ymin)
    {
      Ymin = Y;
    }
    if (Ymax < Y)
    {
      Ymax = Y;
    }
    // display X and Y on serial monitor
    Serial.print("Xmin = ");
    Serial.print(Xmin);
    Serial.print(" X = ");
    Serial.print(X);
    Serial.print(" Xmax = ");
    Serial.print(Xmax);
    Serial.print("| Ymin = ");
    Serial.print(Ymin);
    Serial.print(" Y = ");
    Serial.print(Y);
    Serial.print(" Ymax = ");
    Serial.println(Ymax);
  }
#endif
}

//==============================================================================
// every other function is here

void setWrite_position(uint16_t x, uint16_t y) {
    SPI_sendCommand(ST7789_2A_CASET); // column address set function
	SPI_sendData(x >> 8);	  //Start MSB = XS[15:8]
	SPI_sendData(x & 0x00FF); //Start LSB = XS[ 7:0]
	SPI_sendData(0);		  //End MSB   = XE[15:8] 240-1
	SPI_sendData(240);		  //End LSB   = XE[ 7:0]

	SPI_sendCommand(ST7789_2B_RASET); // row address set function
	y = 319 - y;
	SPI_sendData(y >> 8);	  //Start MSB = YS[15:8]
	SPI_sendData(y & 0x00FF); //Start LSB = YS[ 7:0]
	SPI_sendData(0x01);		  //End MSB   = YE[15:8] 320-1
	SPI_sendData(0x3F);		  //End LSB   = YE[ 7:0]
	
	SPI_sendCommand(ST7789_2C_RAMWR); // write the current position to RAM
}

void fill_LCD(uint8_t R, uint8_t G, uint8_t B)	{
	uint32_t i;
	setWrite_position(0, 319); // set the start position to begin writing data to
    
	for (i = 0; i < (320UL * 240UL); i++) // traverse the entire display
		{
			SPI_sendData(R); //Blue
			SPI_sendData(G); //Green
			SPI_sendData(B); //Red
		}
	}

void ColorBars() // fill LCD with 4 different colored bars
{
	uint16_t number = 4;			   // 4 bars
	uint16_t barHeight = 320 / number; // divide the colors into equal heights
	int i;
	int j;

	setWrite_position(0, 319);
	for (i = 0; i < barHeight; i++) // set the vertical limit as the height of one bar
	{
		for (j = 0; j < 240; j++)
		{
			SPI_sendData(0xFF); // choose the B-G-R code to transfer to the display
			SPI_sendData(0xFF);
			SPI_sendData(0xFF);
		}
	}

	setWrite_position(0, (319 - barHeight)); // update the position to start from the end of the first bar
	for (i = 0; i < barHeight; i++)
	{
		for (j = 0; j < 240; j++)
		{
			SPI_sendData(0xFF);
			SPI_sendData(0x00);
			SPI_sendData(0x00);
		}
	}

	setWrite_position(0, (319-(barHeight * 2))); // update the position to start from the end of the second bar
	for (i = 0; i < barHeight; i++)
	{
		for (j = 0; j < 240; j++)
		{
			SPI_sendData(0x00);
			SPI_sendData(0xFF);
			SPI_sendData(0x00);
		}
	}

	setWrite_position(0, (319-(barHeight * 3))); // update the position to start from the end of the third bar
	for (i = 0; i < barHeight; i++)
	{
		for (j = 0; j < 240; j++)
		{
			SPI_sendData(0x00);
			SPI_sendData(0x00);
			SPI_sendData(0xFF);
		}
	}
}

#define BMP_FLIP	1 // enabling this draws the image the right way up
void show_BMPs_in_root(void)
{
    // these are the colors pulled from the uSD card
	uint8_t R;
	uint8_t G;
	uint8_t B;

	File root_dir;
	root_dir = SD.open("/");
	if (0 == root_dir)
	{
		Serial.println("show_BMPs_in_root: Can't open \"root\"");
		return;
	}
	File bmp_file;

	while (1)
	{

		bmp_file = root_dir.openNextFile();
		if (0 == bmp_file)
		{
			// no more files, break out of while()
			// root_dir will be closed below.
			break;
		}
		// skip directories (what about volume name?)
		if (0 == bmp_file.isDirectory())
		{
			// the file name must include ".BMP"
			if (0 != strstr(bmp_file.name(), ".BMP"))
			{
                Serial.print("size=");
				Serial.println(bmp_file.size()); // print the size of the bmp

				// the BMP must be exactly 230456 long
				if (230456 == bmp_file.size())
				{
					// jump over BMP header, this value is a default for 24-bit bmps
					bmp_file.seek(54);

					// since we are limited in memory, we can not send
					// 240*3 = 720 bytes as is. therefore, split this into three 
					// chunks of 80 pixels each of 80 * 3 = 240 bytes

					// making this static speeds it up slightly (10ms)
					static uint8_t third_of_a_line[80 * 3]; // set the size of the array to 240
					for (uint16_t line = 0; line < 320; line++) // traverse the display in the y direction
					{
						// BMPs store data lowest line first -- bottom up
#if BMP_FLIP						
						setWrite_position(0, (319-line));
#else
						setWrite_position(0, line);
#endif					
                        // move through the 3 chunks of 80 pixels
						for (uint8_t line_section = 0; line_section < 3; line_section++)
						{
							// get a third of the line and store it in the previously defined array
							bmp_file.read(third_of_a_line, 80 * 3);

							// now write this third to the TFT
							SPI_send_pixels(80 * 3, third_of_a_line);
                            // do this until the entire amount of pixels have been sent
						}
					}
			}
			}
		}
		// release the BMP file handle
		bmp_file.close();
		delay(1000);
	}
	// release the root directory file handle
	root_dir.close();
}

void SPI_send_pixels(uint16_t byte_count, uint8_t *data_ptr)
{
	uint8_t subpixel;

	// select the LCD data register
	SET_DC;
	// select the LCD controller
	CLR_CS;

	// load the first byte
	subpixel = *data_ptr;

	while (byte_count)
	{
		// delay(1);
		// send the byte out by writing to the SPI data register
		SPDR = subpixel;
		// whilst transmistting;
		data_ptr++;			  // point to next byte
		subpixel = *data_ptr; // load the next byte
		byte_count--;		  // count the byte increment

		// now that we have done all we can do, wait for the transfer to finish
		while (!(SPSR & _BV(SPIF)))
			; // _BV is defined in the pre-processor and expands to (1 << (SPIF))
	}
    
	// deselect the LCD controller
	SET_CS;
}

#if touch_display 
	#define TS_XL (14)
	#define TS_XR (15)
	#define TS_YD (16)
	#define TS_YU (17)

	#define FIND_MIN_MAX  0
	
	#if(FIND_MIN_MAX)
	uint16_t Xmin=1023;
	uint16_t Xmax=0;
	uint16_t Ymin=1023;
	uint16_t Ymax=0;
	#else
	// copied from the serial console window
	uint16_t Xmin=131;
	uint16_t Xmax=874;
	uint16_t Ymin=99;
	uint16_t Ymax=913;
	#endif

	uint8_t Read_Touch_Screen(uint16_t *x, uint16_t *y)
	{
	// see if there is a touch
	// let YU float, make YD tug high, drive X1 and X2 low.
	// read Y1, if it is near 3.3v (~1024), then the screen is not
	// touched. If it is near ground (~50) then the screen is
	// touched.
	uint16_t touched;
	pinMode(TS_YU, INPUT);
	digitalWrite(TS_YU, HIGH);
	pinMode(TS_YD, INPUT_PULLUP);
	digitalWrite(TS_YD, HIGH);
	pinMode(TS_XL, OUTPUT);
	digitalWrite(TS_XL, LOW);
	pinMode(TS_XR, OUTPUT);
	digitalWrite(TS_XR, LOW);
	touched = analogRead(TS_YU);

	// idle YD as an input
	pinMode(TS_YD, INPUT);
	if (touched < 512)
	{
		// we are touched.
		uint32_t X;
		uint32_t Y;

		// read X. Set a gradient from 0v to 3.3v on X, then
		// read the Y line to get the X contact point.
		// pinMode(TS_YU,INPUT);    //Already set
		// pinMode(TS_YD,INPUT);    //Already set
		// pinMode(TS_XL,OUTPUT);   //Already set
		digitalWrite(TS_XR, HIGH);
		//pinMode(TS_XR,OUTPUT);   //Already set
		//digitalWrite(TS_XL,LOW); //Already set
		X = analogRead(TS_YD); //Could use YU

		// read Y. set a gradient from 0v to 3.3v on Y, then
		// read the X line to get the Y contact point.
		pinMode(TS_XL, INPUT);
		pinMode(TS_XR, INPUT);
		pinMode(TS_YU, OUTPUT);
		digitalWrite(TS_YU, HIGH);
		pinMode(TS_YD, OUTPUT);
		digitalWrite(TS_YD, LOW);
		Y = analogRead(TS_XL); // could use XR

		Serial.print("X: ");
		Serial.println(X);
		Serial.print("Y: ");
		Serial.println(Y);
		// idle the Y pins
		pinMode(TS_YU, INPUT);
		pinMode(TS_YD, INPUT);

		// calculate the pixel values, store in the user's pointers.
		*x = ((X - (uint32_t)Xmin) * 240) / ((uint32_t)Xmax - (uint32_t)Xmin);
		*y = 320 - ((Y - (uint32_t)Ymin) * 320) / ((uint32_t)Ymax - (uint32_t)Ymin);

		// return touched flag
		return (1);
	}
	else
	{
		// not touched. idle the pins that were set to output
		// to detect the touch.
		pinMode(TS_XL, INPUT);
		pinMode(TS_XR, INPUT);
		return (0);
	}
	}

	void touchDemo(void)
	{
	uint16_t x;
	uint16_t y;

	// enable this section for simple touch screen demo
	while (1)
	{
		if (Read_Touch_Screen(&x, &y))
		{
		// touch in upper right corner gets a clear
		if ((300 < x) && (y < 20))
		{
			fill_LCD(0x00, 0x00, 0xFF);
		}
		//touch in upper left corner exits
		if ((x < 20) && (y < 20))
		{
			break;
		}
		// otherwise draw
		LCD_Circle(x, y, 5, 0xFF, 0x00, 0xFF);
		}
		delay(10);
	}
	}
#endif

void Put_Pixel(uint16_t x, uint16_t y, uint8_t R, uint8_t G, uint8_t B)
	{
		setWrite_position(x, y);
		//Write the single pixel's worth of data
		SPI_sendData(B); //Blue
		SPI_sendData(G); //Green
		SPI_sendData(R); //Red
	}

// From: http://en.wikipedia.org/wiki/Midpoint_circle_algorithm
void LCD_Circle(uint16_t x0, uint16_t y0, uint16_t radius,
				uint16_t R, uint16_t G, uint16_t B)
{
	uint16_t x = radius;
	uint16_t y = 0;
	int16_t radiusError = 1 - (int16_t)x;

	while (x >= y)
	{
		// 11 O'Clock
		Put_Pixel(x0 - y, y0 + x, R, G, B);
		// 1 O'Clock
		Put_Pixel(x0 + y, y0 + x, R, G, B);
		// 10 O'Clock
		Put_Pixel(x0 - x, y0 + y, R, G, B);
		// 2 O'Clock
		Put_Pixel(x0 + x, y0 + y, R, G, B);
		// 8 O'Clock
		Put_Pixel(x0 - x, y0 - y, R, G, B);
		// 4 O'Clock
		Put_Pixel(x0 + x, y0 - y, R, G, B);
		// 7 O'Clock
		Put_Pixel(x0 - y, y0 - x, R, G, B);
		// 5 O'Clock
		Put_Pixel(x0 + y, y0 - x, R, G, B);

		y++;
		if (radiusError < 0)
			radiusError += (int16_t)(2 * y + 1);
		else
		{
			x--;
			radiusError += 2 * (((int16_t)y - (int16_t)x) + 1);
		}
	}
}

#define mSwap(a, b, t) \
	{                  \
		t = a;         \
		a = b;         \
		b = t;         \
	}