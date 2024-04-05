#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>

/*
Blink
Turns an LED on for one second, then off for one second, repeatedly.

BOARD: ATtiny1614/1604/814/804/414/404/214/204
Chip: ATtiny1614
Clock Speed: 20MHz
Programmer: jtag2updi (megaTinyCore)

A
Faster
LCD
i2c
interface

*/

#define LED_BUILTIN 0 // PA4 (Pin 2)
#define LED2_BUILTIN 1
#define LED3_BUILTIN 2
#define LED4_BUILTIN PIN_PA3

using Func = uint8_t;
enum : Func
{
  ENDPATTERN,
  WAIT200,
  WAIT50,
  ON1,
  ON2,
  OFF1,
  OFF2
};

Func pat1[] = {ON1, WAIT50, OFF1, WAIT50, ON1, WAIT50, OFF1, WAIT50, ON1, WAIT50, OFF1, WAIT200, ON2, WAIT50, OFF2, WAIT50, ON2, WAIT50, OFF2, WAIT50, ON2, WAIT50, OFF2, WAIT200, ENDPATTERN};

Func pat2[] = {ON1, WAIT50, OFF1, WAIT50, ON2, WAIT50, OFF2, WAIT50, ENDPATTERN};

Func pat3[] = {ON1, WAIT50, OFF1, WAIT200, WAIT200, ON2, WAIT50, OFF2, WAIT200, WAIT200, ENDPATTERN};

Func pat4[] = {ON1, WAIT200, OFF1, ON2, WAIT200, OFF2, ENDPATTERN};

void LCD_Init();
void LCD_Write(char *str, uint8_t line = 1, uint8_t index = 0);
void LCD_Write(uint8_t chr, uint8_t line = 1, uint8_t index = 0);
void LCD_Write(uint8_t *charArray, uint8_t length, uint8_t line = 1, uint8_t index = 0);
void LCD_Send(uint8_t data);
void LCD_Clear();
void LCD_Send(uint8_t data);
void LCD_SendCommand(uint8_t data);
void DoLCD();

volatile uint8_t i2cBuf[22];

// I2C API

using UCommand = uint8_t;
enum : UCommand
{
  NOP,
  LINE1,
  LINE2,
  LINE3,
  LINE4,
  CLEAR,
  BL_ON,
  BL_OFF,
  BL_AUTO,
  CUSTOM_CHAR0,
  CUSTOM_CHAR1,
  CUSTOM_CHAR2,
  CUSTOM_CHAR3,
  CUSTOM_CHAR4,
  CUSTOM_CHAR5,
  CUSTOM_CHAR6,
  CUSTOM_CHAR7,
};

// I2C Slave

#include <Wire.h>

void receiveDataWire(int numBytes)
{ // the Wire API tells us how many bytes
  i2cBuf[0] = numBytes;
  for (uint8_t i = 0; i < numBytes; i++)
  {                              // were received so we can for loop for that
    i2cBuf[i + 1] = Wire.read(); // amount and read the received data
    // MySerial.write(c);                         // to print it to the Serial Monitor
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
  Wire.onReceive(receiveDataWire); // give the Wire library the name of the function

  Wire.onRequest(transmitDataWire); // same as above, but master read event

  Wire.begin(0x21); // join i2c bus with address 0x54
  // MySerial.begin(115200);
}

// I2C Slave end

bool Action(Func func) // Do an action, returns false on ENDPATTERN
{
  if (func == ENDPATTERN)
    return false;
  if (func == ON1)
    digitalWrite(LED_BUILTIN, HIGH);
  if (func == ON2)
    digitalWrite(LED2_BUILTIN, HIGH);
  if (func == OFF1)
    digitalWrite(LED_BUILTIN, LOW);
  if (func == OFF2)
    digitalWrite(LED2_BUILTIN, LOW);
  if (func == WAIT50)
    delay(50);
  if (func == WAIT200)
    delay(200);
  return true;
}

// void DoPattern(Func pat[], size_t len, uint8_t repeats)
void DoPattern(Func pat[], uint8_t repeats)
{
  for (uint8_t repeat = 0; repeat < repeats; repeat++)
  {
    uint8_t x = 0;
    while (Action(pat[x++]))
    {
    }
  }
}

uint8_t intCount;
uint8_t pwmVal = 12;

ISR(TCA0_OVF_vect)
{
  // Your interrupt service routine code here
  // For example, toggle an LED or handle timing-sensitive tasks

  intCount++;
  if (intCount == 32)
    intCount = 0;

  // digitalWrite(LED3_BUILTIN, pwmVal >= intCount);

  digitalWrite(PIN_PB3, pwmVal >= intCount);

  digitalWrite(LED4_BUILTIN, !digitalRead(LED4_BUILTIN)); // results in only 535 Hz...

  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm; // Clear the interrupt flag
}

void setupTimer()
{
  cli(); // Disable global interrupts

  // Configure the timer
  TCA0.SINGLE.CTRLA = 0;                          // Stop the timer
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc; // Set prescaler to 64

  // TCA0.SINGLE.PER = (20000000 / (64 * 10000)) - 1; // Calculate the period for a 10 kHz interrupt rate
  TCA0.SINGLE.PER = 31; // Calculate the period for a 10 kHz interrupt rate

  TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;   // Enable overflow interrupt
  TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm; // Enable the timer

  sei(); // Enable global interrupts
}

// the setup function runs once when you press reset or power the board
void setup()
{
  // TinyI2C.init();
  // Serial.begin(9600);
  /*
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED2_BUILTIN, OUTPUT);
  // pinMode(LED3_BUILTIN, OUTPUT);
  pinMode(LED4_BUILTIN, OUTPUT);
  setupTimer();

  // scanI2C();
  */

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



void loop()
{
  // DoPattern(pat4, 20);
  // DoPattern(pat3, 10);
  // DoPattern(pat2, 10);
  // DoPattern(pat1, 5);
  // delay(10);
  count++;

  // sprintf(line, "Count %d", count);

  LCD_Write("Count: ", 4);
  WriteCount(count);

  DoLCD();
}

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

#define LCD_D4 PIN_PA4
#define LCD_D5 PIN_PA5
#define LCD_D6 PIN_PA2
#define LCD_D7 PIN_PA7
#define LCD_RS PIN_PA1
#define LCD_RW PIN_A6
#define LCD_EN PIN_PA3
#define LCD_BL PIN_PB3

#define LCD_SETCGRAMADDR 0x40

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

void LCD_Send(uint8_t data);
void LCD_SendCommand(uint8_t data);

void DoLCD()
{
  if (!i2cBuf[0])
    return;

  if (i2cBuf[1] >= LINE1 && i2cBuf[1] <= LINE4)
  {
    LCD_Write(i2cBuf + 2, i2cBuf[0] - 1, i2cBuf[1], 0);
    i2cBuf[0] = 0;
    return;
  }

  if (i2cBuf[1] == CLEAR)
    LCD_Clear();
  if (i2cBuf[1] == BL_OFF)
    digitalWrite(LCD_BL, LOW);
  if (i2cBuf[1] == BL_ON)
    digitalWrite(LCD_BL, HIGH);
}

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