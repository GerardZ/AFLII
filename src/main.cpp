#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#include "lcdmod.h"

// I2C Slave

#include <Wire.h>

void DoLCD();

// volatile inline uint8_t i2cBuf[22];

struct I2CBUF
{
  uint8_t len;
  uint8_t command;
  uint8_t data[22];
  uint32_t commandBusy;
};

volatile I2CBUF i2cBuffer;

void receiveDataWire(int numBytes)
{ // the Wire API tells us how many bytes
  i2cBuffer.len = numBytes - 1;
  i2cBuffer.command = Wire.read();
  for (uint8_t i = 0; i < numBytes - 1; i++)
  {
    i2cBuffer.data[i] = Wire.read();
  }

  //while(i2cBuffer.commandBusy > millis()){}
  //if (i2cBuffer.command >= SET_CUSTOM_CHAR0 && i2cBuffer.command <= SET_CUSTOM_CHAR7){
    //delay(30);
    //}
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

/*
// PWM
unsigned int Period = 0xFFFF;

void setup() {                // We will be outputting PWM on PB0
  pinMode(PIN_PB0, OUTPUT);   // PB0 - TCA0 WO0, pin7 on 14-pin parts
  takeOverTCA0();             // This replaces disabling and resettng the timer, required previously.
  TCA0.SINGLE.CTRLB = (TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc);
  TCA0.SINGLE.PER   = Period; // Count all the way up to 0xFFFF
  //                             At 20MHz, this gives ~305Hz PWM
  TCA0.SINGLE.CMP0  = 0;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm; // enable the timer with no prescaler
}

void loop() {
  PWMDemo(150000);  // 150kHz
  PWMDemo(70000);   // 70kHz
  PWMDemo(15000);   // 15kHz
  PWMDemo(3000);    // 3kHz
  PWMDemo(120);     // 120Hz
  PWMDemo(35);      // 35Hz
  PWMDemo(13);      // 13Hz
}

void PWMDemo(unsigned long frequency) {
  setFrequency(frequency);
  setDutyCycle(64); // ~25%
  delay(4000);
  setDutyCycle(128);// ~50%
  delay(4000);
  setDutyCycle(192);// ~75%
  delay(4000);
}

void setDutyCycle(byte duty) {
  TCA0.SINGLE.CMP0 = map(duty, 0, 255, 0, Period);
}

void setFrequency(unsigned long freqInHz) {
  unsigned long tempperiod = (F_CPU / freqInHz);
  byte presc = 0;
  while (tempperiod > 65536 && presc < 7) {
    presc++;
    tempperiod      = tempperiod >> (presc > 4 ? 2 : 1);
  }
  Period            = tempperiod;
  TCA0.SINGLE.CTRLA = (presc << 1) | TCA_SINGLE_ENABLE_bm;
  TCA0.SINGLE.PER   = Period;
}
// end PWM
*/

void setup()
{
  LCD_Init();
  LCD_Write("Hello world!");
  LCD_Write("2nd line...", 2);

  LCD_Write("3rd line...", 3);
  LCD_Send(0);
  LCD_Send(1);
  LCD_Send(2);
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
  if (i2cBuffer.command == NOP)
    return;

  if (i2cBuffer.command >= LINE1 && i2cBuffer.command <= LINE4)
    LCD_Write(i2cBuffer.data, i2cBuffer.len, i2cBuffer.command, 0);

  if (i2cBuffer.command == CLEAR)
    LCD_Clear();

  if (i2cBuffer.command == BL_OFF)
  {
    LCD_Backlight(0);
    //LCD_Write("BL off...", 3);
  }

  if (i2cBuffer.command == BL_ON)
      {
    LCD_Backlight(255);
    //LCD_Write("BL on...", 3);
  }

  if (i2cBuffer.command >= SET_CUSTOM_CHAR0 && i2cBuffer.command <= SET_CUSTOM_CHAR7)
  {
    LCD_createChar(i2cBuffer.command - SET_CUSTOM_CHAR0, const_cast<uint8_t*>(i2cBuffer.data));
    i2cBuffer.commandBusy = millis() + 50;
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
