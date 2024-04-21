#include "lcdmod.h"

/*
LCD-routines, use a AtTiny414 to interface LCD
We use 4-bit communication to preserve pins
Maybe connect contrast to DAC !
Build I2C-slave

Features:
  - check BUSY
  - all working 4x20
  - optimised for speed (writes ca. 1000 lines/sec)
*/

volatile boolean LCDBusy;

byte smiley[] = {
    B00000,
    B10001,
    B00000,
    B00000,
    B10001,
    B01110,
    B00000,
    B00000,
};

byte Heart[] = {
    B00000,
    B01010,
    B11111,
    B11111,
    B01110,
    B00100,
    B00000,
    B00000};

byte Bell[] = {
    B00100,
    B01110,
    B01110,
    B01110,
    B11111,
    B00000,
    B00100,
    B00000};

void DataBusInput()
{
    /*
    Sets D4-D7 as input, then sets LCD_RW high (read).
    */
    
    pinMode(LCD_D4, INPUT); // D4-D7 input
    pinMode(LCD_D5, INPUT);
    pinMode(LCD_D6, INPUT);
    pinMode(LCD_D7, INPUT);

    digitalWrite(LCD_RW, HIGH); // SET READ MODE, set LCD databus as output
}

void DataBusOutput()
{
    /*
    Sets LCD_RW low, then sets D4-D7 as output
    */
    
    digitalWrite(LCD_RW, LOW); // SET WRITE MODE, actually set LCD databus as input.

    pinMode(LCD_D7, OUTPUT); // D4-D7 output
    pinMode(LCD_D6, OUTPUT);
    pinMode(LCD_D5, OUTPUT);
    pinMode(LCD_D4, OUTPUT);
}

void LCD_createChar(uint8_t location, uint8_t charmap[])
{
    /*
    Writes 8 bytes in the character memory @location.
    The LCD has 8 user defined 8-byte memory locations @0x40 - 0x47 in which
    it can store user defined characters

    example:
        LCD_createChar(
                        3,
                        {   B00000,
                            B01010,
                            B11111,
                            B11111,
                            B01110,
                            B00100,
                            B00000,
                            B00000 }
                        );
    // which will create a heart character on location @3
    
    // To send it:
    LCD_Write("This is a heart: ", 2);  // write on line 2, note that cursor stays at last char
    LCD_SendData(3);                    // will actually write char from user memory #3
                        
    */
    
    location &= 0x7; // we only have 8 locations 0-7

    LCD_SendCommand(LCD_SETCGRAMADDR | (location << 3));

    for (int i = 0; i < 8; i++)
    {
        LCD_SendData(charmap[i]); // call the virtual write method
    }
}

bool checkBit(byte targetByte, int position)
{
    return (targetByte & (1 << position)) != 0;
}

void LCD_Clear() // clear Screen
{
    LCD_SendCommand(0x01);
}

void LCD_Backlight(uint8_t value)
{
    if (value != 0)
        digitalWrite(LCD_BL, HIGH);
    else
        digitalWrite(LCD_BL, LOW);
}

void LCD_Init() // Reset, Init & Clear LCD
{
    /* 
    Set appropriate pins in needed IO mode,
    Send init to LCD
    Clear LCD
    */

    pinMode(LCD_RS, OUTPUT);
    pinMode(LCD_EN, OUTPUT);
    pinMode(LCD_BL, OUTPUT);
    pinMode(LCD_RW, OUTPUT);
    DataBusOutput();

    LCD_SendCommand(0x33); // Must initialize to 8-line mode at first
    LCD_SendCommand(0x32); // Then initialize to 4-line mode
    LCD_SendCommand(0x28); // 2 Lines & 5*7 dots
    LCD_SendCommand(0x0C); // Enable display without cursor
    LCD_Clear();

    digitalWrite(LCD_BL, HIGH);

    LCD_createChar(0, smiley);
    LCD_createChar(1, Bell);
    LCD_createChar(2, Heart);
}

void LCD_SetCursor(uint8_t x, uint8_t y)
{
    y &= 3;
    const uint8_t startLine[] = {0x00, 0x40, 0x14, 0x54}; // mem map for lines: ln1: 0x00, ln2: 0x40, ln3: 0x14, ln4: 0x54
    LCD_SendCommand(0x80 + startLine[y] + x);             // Move cursor
}

void LCD_Delay1()
{
    delay(1);
}

void WaitBusy() // wait for busy flag
{
    /*
    Busy Flag (BF) When the busy flag is 1, the HD44780U is in the internal operation mode,
    and the next instruction will not be accepted. When RS = 0 and R/W = 1 (Table 1),
    the busy flag is output to DB7. The next instruction must be written after ensuring that
    the busy flag is 0.

    What is not mentioned here is, the EN needs to be clocked as well and for 4-bit mode TWICE.
    And be aware that you still need to read the lo-nibble after detecting BUSY low.
    */

    LCDBusy = true;

    digitalWrite(LCD_RS, LOW); // To read busy, RS needs to be low

    DataBusInput(); // set bus input

    while (LCDBusy) // read and wait busy
    {
        digitalWrite(LCD_EN, HIGH); // clock in high-nibble, with BUSY
        //LCD_ShortDelay();
        delayMicroseconds(10);

        LCDBusy = digitalRead(LCD_D7);

        digitalWrite(LCD_EN, LOW); 

        Pulse_EN();                 // clock in lo-nibble
    }

    DataBusOutput(); // Set bus output again
}

void LCD_ShortDelay()
{
    return;
}

void Pulse_EN() // Clock-in data
{
    digitalWrite(LCD_EN, HIGH);
    //LCD_ShortDelay();
    delayMicroseconds(10);
    digitalWrite(LCD_EN, LOW);
}

void LCD_SendCommand(uint8_t data) // Send a command to LCD, difference with only data is that RS = 0
{
    digitalWrite(LCD_RS, LOW);
    LCD_Send(data);
    WaitBusy(); // wait for previous command
}

void LCD_SendData(uint8_t data) // Send data to LCD, RS = 1
{
    digitalWrite(LCD_RS, HIGH);
    LCD_Send(data);
    WaitBusy(); // wait for previous command
}

void LCD_Send(uint8_t data) // Send data to LCD
{
    digitalWrite(LCD_RW, LOW);

    digitalWrite(LCD_D4, checkBit(data, 4)); // HIGH-nibble
    digitalWrite(LCD_D5, checkBit(data, 5));
    digitalWrite(LCD_D6, checkBit(data, 6));
    digitalWrite(LCD_D7, checkBit(data, 7));
    Pulse_EN();

    digitalWrite(LCD_D4, checkBit(data, 0)); // LOW-nibble
    digitalWrite(LCD_D5, checkBit(data, 1));
    digitalWrite(LCD_D6, checkBit(data, 2));
    digitalWrite(LCD_D7, checkBit(data, 3));
    Pulse_EN();
}

void LCD_Write(uint8_t *charArray, uint8_t length, uint8_t line = 1, uint8_t index = 0)
{
    LCD_SetCursor(index, line - 1);
    for (int i = 0; i < length; i++)
    {
        LCD_SendData(charArray[i]);
    }
}

void LCD_Write(char *str, uint8_t line = 1, uint8_t index = 0)
{
    LCD_SetCursor(index, line - 1);
    for (int i = 0; str[i] != '\0'; i++)
    {
        LCD_SendData(str[i]);
    }
}

void LCD_Write(uint8_t chr, uint8_t line = 1, uint8_t index = 0)
{
    LCD_SetCursor(index, line - 1);
    LCD_SendData(chr);
}