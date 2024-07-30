# AFLII

## A Faster Lcd I2c Interface

### **So what is this ?**

This is a replacement for the PCF8574 LCD-interface board, which is pin but not API compatible.

It is faster and less cpu consuming for the host because the used AtTiny414 takes care of low-level data handling, besides BUSY-flag is used instead of wait-routines which allows for faster and more easy writing the display.

### **And why should i use this ?**

For me this was a getting-to-know the AtTiny414 project, but it also has some advantages above the PCF8574-solution.

Pro\`s:

*   faster, steering the LCD module is off-loaded from the host. Benchmarked @>300 lines/second
*   more efficient by using BUSY instead of wait
*   easy API, in essence just push line number and string over i2c. Uses less memory and cpu on host
*   cheaper (at least @Mouser, a AtTiny814 costs around 55 cents, where a PCF8574 does 120 cents)
*   Backlight dimming !   
*   you could incorporate some custom routines
*   pin free for user
*   you might learn from this :)

Con\`s:

*   the rest of the world uses PCF8574
*   there\`s no ready-build hardware available
*   API differs from the PCF solution

Todo:

*   The WaitBusy should hold I2C communication by clock stretching. We now still need a delay @host side.
*   Better PCB design
*   More & clearer examples