#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#include "lcdmod.h"

// I2C Slave

#include <Wire.h>

void DoLCD();

struct I2CBUF
{
  volatile uint8_t len;
  volatile uint8_t command;
  volatile uint8_t data[20];
};

volatile I2CBUF i2cBuffer;

void receiveDataWire(int numBytes)
{
  i2cBuffer.len = numBytes - 1; // -1 byte0 is command
  i2cBuffer.command = Wire.read();

  for (uint8_t i = 0; (i < numBytes - 1) && (i < 20); i++)
  {
    i2cBuffer.data[i] = Wire.read();
  }

  // WaitBusy();
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
  Wire.begin(0x21);                 // join i2c bus with address 0x54
}

// I2C Slave end



void demo(){
   // just for demonstration actually...
  LCD_createChar(0, Smiley);
  LCD_createChar(1, Bell);
  LCD_createChar(2, Heart);
  
  LCD_Write("Hello world!");
  LCD_Write("2nd line...", 2);

  LCD_Write("3rd line...", 3);
  LCD_SendData(0);
  LCD_SendData(1);
  LCD_SendData(2);
  LCD_Write("4th line...", 4);

}

void setup()
{
  LCD_Init();

  setupI2C();

  SetBacklightDimInt(10);
  //SetBacklightDimTimer(100);

  demo();
}

// uint8_t line[16];
volatile uint16_t count = 0;

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
    LCD_SendData(dec);
  }
  LCD_SendData(count + 0x30);
}

void DoLCD()
{
  if (i2cBuffer.command == NOP)
    return;

  if (i2cBuffer.command >= LINE1 && i2cBuffer.command <= LINE4)
    LCD_Write(i2cBuffer.data, i2cBuffer.len, i2cBuffer.command, 0);

  if (i2cBuffer.command == CLEAR)
  {
    LCD_Clear();
    // delay(50);
  }

  if (i2cBuffer.command == BL_OFF)
  {
    LCD_Backlight(0);
    // LCD_Write("BL off...", 3);
  }

  if (i2cBuffer.command == BL_ON)
  {
    LCD_Backlight(255);
    // LCD_Write("BL on...", 3);
  }

  if (i2cBuffer.command >= SET_CUSTOM_CHAR0 && i2cBuffer.command <= SET_CUSTOM_CHAR7)
  {
    LCD_createChar(i2cBuffer.command - SET_CUSTOM_CHAR0, const_cast<uint8_t *>(i2cBuffer.data));
    // delay(10);
  }

  i2cBuffer.command = NOP;
}

void loop()
{
  count++;

  LCD_Write("Count: ", 4);
  WriteCount(count);

  

  DoLCD();
}
