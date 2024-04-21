#pragma once

#include <Arduino.h>

using UCommand = uint8_t;
enum : UCommand
{
  NOP,
  LINE1,
  LINE2,
  LINE3,
  LINE4,
  RAW,
  CLEAR,
  BL_ON,
  BL_OFF,
  BL_AUTO,
  SET_CUSTOM_CHAR0,
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








