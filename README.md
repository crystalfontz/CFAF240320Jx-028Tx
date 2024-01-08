# CFAF240320Jx-028Tx Demonstration Code

This is Arduino sample code for the CFAF240320Jx-028Tx family of displays. These displays are 2.8" TFTs which use IPS technology enabling wide viewing angles and use the [Sitronix ST7789V](https://www.crystalfontz.com/controllers/Sitronix/ST7789V/) LCD controller. These displays are capable of 8/9/16/18 bit parallel, 3 or 4-Wire SPI and DOTCLK RGB. In order to interface with the displays, a 45 pin, 0.3mm pitch ZIF to header breakout board will be required. Both non-touch and resistive touch options are available for this lineup of displays.
 
## Connection Guide
```
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
(Micro SD card connection is optional)
```
## Display information
Here are links to our active displays:\
[CFAF240320J0-028TN](https://www.crystalfontz.com/product/cfaf240320j0028tn)\
[CFAF240320J0-028TR](https://www.crystalfontz.com/product/cfaf240320j0028tr)

For more information about other TFT offerings, please see our full list [here](https://www.crystalfontz.com/c/tft-lcd-displays/25).