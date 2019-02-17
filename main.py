# Open Source Air Quality sensor project
# MIT license; Copyright (c) 2019 Daniel Near (dan.e.near@gmail.com)
# Version 0.1 beta (2019/02/17)
#
# This code developed to run on Pycom LoPy4
# This version works with several Adafruit sensor PCBS
# MPL3115A2 Barometric pressure
# HTU21D Temp/Humidity sensor
# TSL2561 Ambient and IR Light sensor
# Note Adafruit sensors need both Vin and 3v0 pins tied together to 3v3 or they won't have clean signals
# I used a 1.6k Pullup resistor each on SDA and SCL to 3v3
# Libraries for these devices were collected online and modified
# to work with Pycom's Micropython variant
# abpkeys.py was included for code legibility but is zeroed out.  Don't publish your private keys!
# to get keys for these three values, register with TheThingsNetwork, create an application
# and add a unique device for each Pycom assembly you build. 
# Configure that device to connect via ABP.
# You will need to be near a LoRaWAN Gateway or purchase your own
 
from htu21d import HTU21D
import time
import math 
import machine
# Create library object using our Bus I2C port
#i2c = busio.I2C(board.SCL, board.SDA)
#from machine import I2C, Pin
#i2c = I2C(0, pins=("P9","P10"))
#i2c.init(I2C.MASTER, baudrate=9600)

     
 # main.py -- put your code here!
from network import LoRa
import socket
import ubinascii
import struct
import time
import os
import math
import abpkeys
import pycom
import ustruct
from machine import UART, I2C, Pin
from htu21d import HTU21D
from MPL3115A2 import MPL3115A2,ALTITUDE,PRESSURE
import TSL2561
sensor = HTU21D()
#from machine import I2C, Pin #added for si7021 rh sensor
#f = open('/flash/test.py', 'a') #create file (append mode)
#f.write("Start\n")
#f.close()
#from si7021 import SI7021 #added for si7021 rh sensor

uart = UART(1, 9600)                         # init with given baudrate
uart.init(9600, bits=8, parity=None, stop=1) # init with given parameters

#initialize i2c to link to rh sensor
i2c = I2C(0, pins=("P9","P10"))
i2c.init(I2C.MASTER, baudrate=100000)
#tsl = adafruit_tsl2561.TSL2561(i2c)
sensor = HTU21D()

tslDev = TSL2561.device()
tslDev.init()
# Initialise LoRa in LORAWAN mode.
# Please pick the region that matches where you are using the device:
# Asia = LoRa.AS923
# Australia = LoRa.AU915
# Europe = LoRa.EU868
# United States = LoRa.US915
lora = LoRa(mode=LoRa.LORAWAN, region=LoRa.US915)

# create an ABP authentication params
#my keys are stored in abpkeys.py then linked here so I have the same source running 
#on multiple devices
dev_addr=abpkeys.dev_addr
nwk_swkey=abpkeys.nwk_swkey
app_swkey=abpkeys.app_swkey
#dev_addr = struct.unpack(">l", ubinascii.unhexlify('xxxxxxxx'))[0]
#nwk_swkey = ubinascii.unhexlify('xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx')
#app_swkey = ubinascii.unhexlify('xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx')
for i in range(0, 71):
    lora.remove_channel(i)
print('Removed default channels')
time.sleep(1)
    
    # Set US ISM 915 channel plan for TTN US
lora.add_channel(0, frequency=903900000, dr_min=0, dr_max=3)
lora.add_channel(1, frequency=904100000, dr_min=0, dr_max=3)
lora.add_channel(2, frequency=904300000, dr_min=0, dr_max=3)
lora.add_channel(3, frequency=904500000, dr_min=0, dr_max=3)
lora.add_channel(4, frequency=904700000, dr_min=0, dr_max=3)
lora.add_channel(5, frequency=904900000, dr_min=0, dr_max=3)
lora.add_channel(6, frequency=905100000, dr_min=0, dr_max=3)
lora.add_channel(7, frequency=905300000, dr_min=0, dr_max=3)
#channel	8:	903900000	hz	min_dr	0	max_dr	3				
#channel	9:	904100000	hz	min_dr	0	max_dr	3				
#channel	10:	904300000	hz	min_dr	0	max_dr	3				
#channel	11:	904500000	hz	min_dr	0	max_dr	3				
#channel	12:	904700000	hz	min_dr	0	max_dr	3				
#channel	13:	904900000	hz	min_dr	0	max_dr	3				
#channel	14:	905100000	hz	min_dr	0	max_dr	3				
#channel	15:	905300000	hz	min_dr	0	max_dr	3				

print('US channels set')
time.sleep(1)


print('restoring LoRa NVRAM')
lora.nvram_restore()
print('Joining LoRa via ABP')
# join a network using ABP (Activation By Personalization)
lora.join(activation=LoRa.ABP, auth=(dev_addr, nwk_swkey, app_swkey))

# create a LoRa socket
s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)

# set the LoRaWAN data rate
s.setsockopt(socket.SOL_LORA, socket.SO_DR, 3)

#Let's check whether the si7021 is attached and enable further code if it responds
print('check i2c')
#si7021presence=True
#temperature=-255
buffer=i2c.scan()
#print(buffer)
print(buffer)
#f = open('/flash/test.py', '+a') #create file (append mode)
#f.write("Buffer=\n")
#f.write(bytes(buffer[1])
#f.close()

#try:
#    temperature=int(sensor.temperature())
#    print('temperature = ',temperature)
#except NameError as msg:
#    print(msg)
#    si7021presence=False
#if( temperature !=-255):
#    print('temperature reported = ',temperature)
#print('si7021 presence = ',si7021presence)
#The first 5 seconds of dust data always seems irregular, so let's delay and clear the buffer
data = uart.readall()
time.sleep(10)
data= uart.readall()
time.sleep(1)# then give it another second for the next good byte to come in.
while (1):
    readytosend=False
    print('polling Dust Sensor')
    data = uart.readall()# get everythitng from the serial buffer
    if data is not None:#make sure it'st not an empty buffer
        bytestream=data.split(b'BM')[1][0:31]# HPMA sends 32 byte packets that start with \x42\x4D ('BM')
        if len(bytestream)==30:#check tthat we actually got 30 bytes (excluding the data we split out)
            if bytestream[0:2]==b'\x00\x1c':#HPMA packet is always 28 data bytes(excluding start header and checksum)
                checksum=143#since split command removed the \x42\x4D, add them back to get correct checksum
                validated=0
                for i in range(0,28):
                    checksum+=bytestream[i]#cycle thru the data values and add them up
                validated=int(bytestream[28])*256+int(bytestream[29])#read checksum bytes from packet and make them a 2-byte integer
                if checksum==validated:# both values should match, else we have a corrupt packet
                    pm25=int(bytestream[4])*256+int(bytestream[5])#these two bytes represent the particulate detected that's 2.5um large
                    pm10=int(bytestream[6])*256+int(bytestream[7])#these two bytes represent the particulate detected that's 10um large
                    print('pm2.5 = ',pm25)
                    print('pm10 = ',pm10)
                    readytosend=True
                else:
                    print('checksum failed')
            else:
                print('invalid packet length')
        else:
            print('incomplete packet')
    #if(si7021presence==True):
    print(i2c.scan())
    print('poll temp')
    Temperature=sensor.temperature
    print('poll RH')
    RH=sensor.relative_humidity
    print('calc Dew')
    h = (math.log(RH, 10) - 2) / 0.4343 + (17.62 * Temperature) / (243.12 + Temperature)
    dp=243.12 * h / (17.62 - h)    
    #dewpoint=int(sensor.dew_point())
    print('poll pressure')
    mpp = MPL3115A2(mode=PRESSURE)# Returns pressure in Pa. Mode may also be set to ALTITUDE, returning a value in meters
    baro=mpp.pressure()/33.86389
    print(i2c.scan())
    lux = tslDev.getSensorDataRaw()
    broadlum=lux['lumB']
    IRLum=lux['lumIR']
    #print('Final Broadband: ' + str(lum['lumB']))
    #print('Final IR: ' + str(lum['lumIR']))

    if readytosend==True:
        # make the socket blocking
        # (waits for the data to be sent and for the 2 receive windows to expire)
        s.setblocking(True)
        print("Sending data!")
        # send some data
        # s.send(bytes([0x01, 0x02, 0x03]))
        #if(si7021presence==True):
        tempstruct=ustruct.pack('>H',int(Temperature))
        rhstruct=ustruct.pack('>H',int(RH))
        dewstruct=ustruct.pack('>H',int(dp))
        presstruct=ustruct.pack('>H',int(baro))
        broadlumstruct=ustruct.pack('>H',lux['lumB'])
        IRlumstruct=ustruct.pack('>H',lux['lumIR'])

        s.send(bytes([bytestream[4], bytestream[5], bytestream[6],bytestream[7],tempstruct[0],tempstruct[1],rhstruct[0],rhstruct[1],dewstruct[0],dewstruct[1],presstruct[0],presstruct[1],broadlumstruct[0],broadlumstruct[1],IRlumstruct[0],IRlumstruct[1]]))#,relativehumid,dewpoint]))
        #else:    
        #    s.send(bytes([bytestream[4], bytestream[5], bytestream[6],bytestream[7]]))
           
        pybytes.send_signal(0, (bytestream[4]<<8)+bytestream[5])
        time.sleep(2)
        pybytes.send_signal(1, (bytestream[6]<<8)+bytestream[7])
        time.sleep(2)
        pybytes.send_signal(2, '%.1f'%Temperature)
        time.sleep(2)
        pybytes.send_signal(3, int(RH))
        time.sleep(2)
        pybytes.send_signal(4, '%.1f'%dp)
        time.sleep(2)
        pybytes.send_signal(5, '%.2f'%(baro/100))
        time.sleep(2)
        pybytes.send_signal(6, lux['lumB'])
        time.sleep(2)
        pybytes.send_signal(7, lux['lumIR'])
        time.sleep(2)
       #pybytes.send_virtual_pin_value(False,2,100)
        #time.sleep(10)

    print('saving LoRa NVRAM')
    lora.nvram_save()
    # make the socket non-blocking
    # (because if there's no data received it will block forever...)
    s.setblocking(False)
    print("receiving data!")
    # get any data received (if any...)
    data = s.recv(64)
    print(data)
    print("Go to sleep 5 minutes!")
    time.sleep(284)    
