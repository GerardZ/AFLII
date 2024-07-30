#pragma once

#include <Arduino.h>

using UCommand = uint8_t;
enum : UCommand
{
  NOP,                // NOP also used to indicate no command has been received
  LINE1,              // set cursor line 1, subsequenting bytes write that line
  LINE2,              // same line 2
  LINE3,              // same line 3
  LINE4,              // same line 4
  RAW,
  CLEAR,              // clear lcd
  BL_ON,              // backlight on
  BL_OFF,             // backlight off
  BL_SET,             // backlight set dim 0-255 in next byte, 1 subsequent byte expected
  SET_CUSTOM_CHAR0,   // define cusom char0, 8 subsequent bytes expected
  SET_CUSTOM_CHAR1,
  SET_CUSTOM_CHAR2,
  SET_CUSTOM_CHAR3,
  SET_CUSTOM_CHAR4,
  SET_CUSTOM_CHAR5,
  SET_CUSTOM_CHAR6,
  SET_CUSTOM_CHAR7,
};

#define LCD_D4 PIN_PA4
#define LCD_D5 PIN_PA5
#define LCD_D6 PIN_PA2
#define LCD_D7 PIN_PA7
#define LCD_RS PIN_PA1
#define LCD_RW PIN_A6
#define LCD_EN PIN_PA3
#define LCD_BL PIN_PB3

#define LCD_SETCGRAMADDR 0x40

#define LCD_CLEAR 0x01           // from https://dawes.wordpress.com/2010/01/05/hd44780-instruction-set/
#define LCD_HOME  0x02
#define LCD_Mode_8Bit 0x33
#define LCD_Mode_4Bit 0x32
#define LCD_Mode_2Line_5x7 0x28
#define LCD_On_CursorOff 0x0C
#define LCD_On_CursorSteady 0x0E
#define LCD_On_CursorBlink 0x0F


#define LCD_SetCursorPosition 0x80

#define LCD_line0_addr 0x00
#define LCD_line1_addr 0x40
#define LCD_line2_addr 0x14
#define LCD_line3_addr 0x54


void LCD_Init();
void LCD_Write(char *str, uint8_t line = 1, uint8_t index = 0);
void LCD_Write(uint8_t chr, uint8_t line = 1, uint8_t index = 0);
void LCD_Write(uint8_t *charArray, uint8_t length, uint8_t line = 1, uint8_t index = 0);
void LCD_Send(uint8_t data);
void LCD_Clear();
void LCD_SendCommand(uint8_t data);
void LCD_SendData(uint8_t data);
void LCD_Backlight(uint8_t value);
void LCD_createChar(uint8_t location, uint8_t charmap[]);
void WaitBusy();
void Pulse_EN();
void LCD_ShortDelay();
void SetBacklightDim(uint8_t dimValue);

extern byte Smiley[];
extern byte Heart[];
extern byte Bell[];











