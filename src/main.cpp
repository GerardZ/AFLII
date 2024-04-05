#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#include "lcdmod.h"

// I2C Slave

#include <Wire.h>

void DoLCD();

//volatile inline uint8_t i2cBuf[22];

struct I2CBUF{
  uint8_t len;
  uint8_t command;
  uint8_t data[22];
};

volatile I2CBUF i2cBuffer;

void receiveDataWire(int numBytes)
{ // the Wire API tells us how many bytes
  i2cBuffer.len = numBytes -1;
  i2cBuffer.command = Wire.read();
  for (uint8_t i = 0; i < numBytes-1; i++){
    i2cBuffer.data[i] = Wire.read();
  }
}

void transmitDataWire()
{
#if !defined(MILLIS_USE_TIMERNONE)
  uint32_t ms = millis();
#else
  uint32_t ms = 123456789UL; // placeholder - there's no millis to send because it's disabled.
#endif
  Wire.write((uint8_t)ms);
  Wire.write((uint8_t)(ms >> 8));
  Wire.write((uint8_t)(ms >> 16));
  Wire.write((uint8_t)(ms >> 24));
}

void setupI2C()
{
  Wire.onReceive(receiveDataWire);  // give the Wire library the name of the function, this is handled in interrupt so keep it short
  Wire.onRequest(transmitDataWire); // same as above, but master read event, this is handled in interrupt so keep it short
  Wire.begin(0x21); // join i2c bus with address 0x54
}

// I2C Slave end


void setup()
{
  LCD_Init();
  LCD_Write("Hello world!");
  LCD_Write("2nd line...", 2);
  LCD_Send(0);
  LCD_Send(1);
  LCD_Send(2);
  LCD_Write("3rd line...", 3);
  LCD_Write("4th line...", 4);

  setupI2C();
}

uint8_t line[16];
uint16_t count = 0;

void WriteCount(uint16_t inCount)
{
  const int q[] = {10000, 1000, 100, 10, 1};
  uint16_t count;
  uint8_t zero = ' ';
  count = inCount;
  uint8_t dec;

  for (uint8_t i = 0; i < 4; i++)
  {
    if (count >= q[i])
    {
      dec = count / q[i];
      count -= dec * q[i];
      dec += 0x30; // convert number to asc value
      zero = '0';
    }
    else
      dec = zero;
    LCD_Send(dec);
  }
  LCD_Send(count + 0x30);
}

void DoLCD()
{
  if (!i2cBuffer.len)
    return;

  if (i2cBuffer.command >= LINE1 && i2cBuffer.command <= LINE4)
      LCD_Write(i2cBuffer.data, i2cBuffer.len, i2cBuffer.command, 0);
  
  if (i2cBuffer.command == CLEAR)
    LCD_Clear();

  if (i2cBuffer.command == BL_OFF)
    LCD_Backlight(0);

  if (i2cBuffer.command == BL_ON)
    LCD_Backlight(1);

  i2cBuffer.len = 0;
}

void loop()
{
  count++;

  LCD_Write("Count: ", 4);
  WriteCount(count);

  DoLCD();
}

