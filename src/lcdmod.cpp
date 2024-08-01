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

volatile int8_t pwmValue = 15;
volatile int8_t pwmCount;

// some demo characters
byte Smiley[] = {
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

ISR(TCA0_OVF_vect)
{ // backlight ISR
    pwmCount++;
    if (pwmCount > 31)
        pwmCount = 0;

    digitalWrite(LCD_BL, pwmValue > pwmCount);

    // Clear the interrupt flag
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}

void setupBLInterrupt()
{
    // Disable interrupts
    cli();

    // Set the TCA0 to Normal mode (by default it's in Split mode)
    TCA0.SINGLE.CTRLD = 0;
    TCA0.SINGLE.CTRLB = 0;                    // Normal mode
    TCA0.SINGLE.CTRLC = 0;                    // No force compare
    TCA0.SINGLE.CTRLECLR = TCA_SINGLE_CMD_gm; // No command

    // Set the period (8 MHz / 64 prescaler / 1000 Hz - 1)
    TCA0.SINGLE.PER = (8000000 / 64 / 3200) - 1;

    // Enable overflow interrupt
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;

    // Set the prescaler and enable the timer
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc | TCA_SINGLE_ENABLE_bm;

    // Enable global interrupts
    sei();
}

void SetBacklightDimInt(uint8_t dimValue)
{
    pwmValue = dimValue / 8;
}

void setupTimerPWM2()
{

    cli(); // Disable interrupts
    // TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc; // Set the waveform generation mode to Single Slope PWM

    // Enable Compare Channel 2 (WO2)
    // TCA0.SINGLE.CTRLB |= TCA_SINGLE_CMP2EN_bm;

    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP2EN_bm;

    TCA0.SINGLE.EVCTRL &= ~(TCA_SINGLE_CNTEI_bm);

    // Set the period (top value)
    TCA0.SINGLE.PERBUF = 0xFF;

    // Set the duty cycle for Compare Channel 2 (50% duty cycle)
    TCA0.SINGLE.CMP2 = 0x80;

    // Select the clock source and start the timer (divided by 64)
    // TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc | TCA_SINGLE_ENABLE_bm;
    TCA0.SINGLE.CTRLA = 7;

    // Enable interrupts
    sei();

    PORTA.DIRSET = PIN2_bm;
}

#if defined(MILLIS_USE_TIMERA0) || defined(__AVR_ATtinyxy2__)
#error "This sketch takes over TCA0, don't use for millis here.  Pin mappings on 8-pin parts are different"
#endif

void setupTimerPWM()
{
    // PA2...

    
    PORTA.DIRSET = PIN2_bm;

    // Disable interrupts
    cli();

    takeOverTCA0();

    // Set the waveform generation mode to Single Slope PWM
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc;

    // Enable Compare Channel 2 (WO2)
    TCA0.SINGLE.CTRLB |= TCA_SINGLE_CMP2EN_bm;

    // Set the period (top value)
    TCA0.SINGLE.PER = 0xFF;

    // Set the duty cycle for Compare Channel 2 (50% duty cycle)
    TCA0.SINGLE.CMP2 = 0x80;

    // Select the clock source and start the timer (divided by 64)
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc | TCA_SINGLE_ENABLE_bm;

    // Enable interrupts
    sei();
}

void SetBacklightDimTimer(uint8_t dimValue)
{
    TCA0.SINGLE.CMP2 = dimValue;
}

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
    LCD_SendCommand(LCD_CLEAR);
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

    LCD_SendCommand(LCD_Mode_8Bit);      // Must initialize to 8-line mode at first
    LCD_SendCommand(LCD_Mode_4Bit);      // Then initialize to 4-line mode
    LCD_SendCommand(LCD_Mode_2Line_5x7); // 2 Lines & 5*7 dots
    LCD_SendCommand(LCD_On_CursorOff);   // Enable display without cursor
    LCD_Clear();

    setupBLInterrupt();
    //setupTimerPWM();

    //analogWrite(LCD_BL, 18); // 50% duty cycle
}

void LCD_SetCursor(uint8_t x, uint8_t y)
{
    y &= 3;
    const uint8_t startLine[] = {LCD_line0_addr, LCD_line1_addr, LCD_line2_addr, LCD_line3_addr}; // mem map for lines: ln1: 0x00, ln2: 0x40, ln3: 0x14, ln4: 0x54
    LCD_SendCommand(LCD_SetCursorPosition + startLine[y] + x);                                    // Move cursor
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
        // LCD_ShortDelay();
        delayMicroseconds(10);

        LCDBusy = digitalRead(LCD_D7);

        digitalWrite(LCD_EN, LOW);

        Pulse_EN(); // clock in lo-nibble
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
    // LCD_ShortDelay();
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