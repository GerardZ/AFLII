import smbus2
import time
import datetime

from zoneinfo import ZoneInfo

from enum import Enum, auto

class UCommand(Enum):
    "NOP",
    "LINE1",
    "LINE2",
    "LINE3",
    "LINE4",
    "RAW",
    "CLEAR",
    "BL_ON",
    "BL_OFF",
    "BL_AUTO",
    "CUSTOM_CHAR0",
    "CUSTOM_CHAR1",
    "CUSTOM_CHAR2",
    "CUSTOM_CHAR3",
    "CUSTOM_CHAR4"
    "CUSTOM_CHAR5",
    "CUSTOM_CHAR6",
    "CUSTOM_CHAR7"

# Define the I2C bus. Raspberry Pi typically uses 1, but some models use 0.
bus_number = 1
bus = smbus2.SMBus(bus_number)
busStatus = True

# Define the I2C address of the slave device
device_address = 0x21  # Replace 0x10 with your device's address

plus = [  0b11111,
            0b11011,
            0b10001,
            0b11011,
            0b11111,
            0,0,0]
            
arrow = [   0b00000,
            0b00100,
            0b01110,
            0b11111,
            0b00100,
            0b00100,
            0,0]
            
test = [    0b01110,
            0b10001,
            0b10001,
            0b10001,
            0b01110,
            0b00100,
            0b00100,
           0]
            

def Write_I2C(address, command, string = ""):
    arr = list(bytearray(string, 'utf-8'))
    WriteRawArr_I2C(address, command, arr)
   

def WriteRawArr_I2C(address, command, arr):
    global busStatus
    global bus
    try:
        if not busStatus:
            bus.close()
            bus = smbus2.SMBus(bus_number)
            busStatus = True
        bus.write_i2c_block_data(address, command, arr)
    except:
        busStatus = False
        print("Bus error...")
        bus.close()  # Ensure the bus is closed
    time.sleep(0.1)


Write_I2C(device_address, 6) # clear
time.sleep(0.5)
WriteRawArr_I2C(device_address, 17, plus)
time.sleep(0.5)
WriteRawArr_I2C(device_address, 16, arrow)
time.sleep(0.5)
WriteRawArr_I2C(device_address, 15, test)
time.sleep(0.5)
Write_I2C(device_address, 3, "Char enzo : " + chr(7) + chr(6)+ chr(5))


# Send bytes over I2C
#for byte in bytes_to_send:
#bus.write_i2c_block_data(device_address, 2, bytes_to_send)
Write_I2C(device_address, 2, "Dit moet werken.     ")
#time.sleep(0.01)  # Short delay to ensure the slave can process the data

def GetTimeString(curTime):
    return curTime.strftime("%H:%M:%S %Z")

def GetTime():
    return datetime.datetime.now(ZoneInfo("Europe/Amsterdam"))

while True:

    curTime = GetTime()

    timeString = GetTimeString(curTime)
    print(timeString)

    dateString = datetime.date.today().strftime('%d-%m-%Y')
    #bytes_to_send = list(bytearray(timeString, 'utf-8'))
    #bus.write_i2c_block_data(device_address, 1, bytes_to_send) 
    Write_I2C(device_address, 1, timeString)
    Write_I2C(device_address, 2, dateString)
    while (curTime.second == GetTime().second):
        pass

        


