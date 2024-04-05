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

void LCD_createChar(uint8_t location, uint8_t charmap[])
{
    location &= 0x7; // we only have 8 locations 0-7

    LCD_SendCommand(LCD_SETCGRAMADDR | (location << 3));
    delayMicroseconds(30);

    for (int i = 0; i < 8; i++)
    {
        LCD_Send(charmap[i]); // call the virtual write method
        delayMicroseconds(40);
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
    if (value)
        digitalWrite(LCD_BL, HIGH);
    else
        digitalWrite(LCD_BL, LOW);
}

void LCD_Init() // Reset, Init & Clear LCD
{
    pinMode(LCD_D4, OUTPUT);
    pinMode(LCD_D5, OUTPUT);
    pinMode(LCD_D6, OUTPUT);
    pinMode(LCD_D7, OUTPUT);
    pinMode(LCD_RS, OUTPUT);
    pinMode(LCD_EN, OUTPUT);
    pinMode(LCD_BL, OUTPUT);
    pinMode(LCD_RW, OUTPUT);

    digitalWrite(LCD_RW, LOW);

    LCD_SendCommand(0x33); // Must initialize to 8-line mode at first
    LCD_SendCommand(0x32); // Then initialize to 4-line mode
    LCD_SendCommand(0x28); // 2 Lines & 5*7 dots
    LCD_SendCommand(0x0C); // Enable display without cursor
    LCD_Clear();
    delay(2);

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

void LCD_SendCommand(uint8_t data) // Send a command to LCD, difference with only data is that RS = 0
{
    digitalWrite(LCD_RS, LOW);
    LCD_Send(data);
    digitalWrite(LCD_RS, HIGH);
}

void LCD_Delay1()
{
    delay(1);
}

void WaitBusy() // wait for busy flag
{
    pinMode(LCD_D7, INPUT);     // D7 IS busy flag when in read mode
    digitalWrite(LCD_RW, HIGH); // SET READ MODE

    while (!digitalRead(LCD_D7)) // read and wait busy
    {
    }

    digitalWrite(LCD_RW, LOW); // Set write again
    pinMode(LCD_D7, OUTPUT);   // Set D7 as output
}

volatile void LCD_ShortDelay()
{
    return;
}

void Pulse_EN() // Clock-in data
{
    digitalWrite(LCD_EN, HIGH);
    LCD_ShortDelay();
    digitalWrite(LCD_EN, LOW);
}

void LCD_Send(uint8_t data) // Send data to LCD
{
    WaitBusy(); // wait for previous command

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
        LCD_Send(charArray[i]);
    }
}

void LCD_Write(char *str, uint8_t line = 1, uint8_t index = 0)
{
    LCD_SetCursor(index, line - 1);
    for (int i = 0; str[i] != '\0'; i++)
    {
        LCD_Send(str[i]);
    }
}

void LCD_Write(uint8_t chr, uint8_t line = 1, uint8_t index = 0)
{
    LCD_SetCursor(index, line - 1);
    LCD_Send(chr);
}