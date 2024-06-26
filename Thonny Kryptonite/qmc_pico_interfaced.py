import machine as mc
import sys
import math
from time import sleep  #import sleep

pinSDA = mc.Pin(20)
pinSCL = mc.Pin(21)
QMC5883L_ADDR = 0x0D
i2c = mc.I2C(0,freq=2000000, scl=pinSCL, sda=pinSDA)
devices = i2c.scan()
if not (QMC5883L_ADDR in devices):
    print("Not found GY-271 (QMC5883L)!")
    sys.exit(1)

############## Register Location
RegCTRL1 = 0x09 # Control Register--> MSB(OSR:2,RNG:2,ODR:2,MODE:2)LSB
RegCTRL2 = 0x0A # Control Register2--> MSB(Soft_RS:1,Rol_PNT:1,none:5,INT_ENB:1)LSB
RegFBR   = 0x0B # SET/RESET Period Register--> MSB(FBR:8)LSB
RegXLo   = 0x00
RegXHi   = 0x01
RegYLo   = 0x02
RegYHi   = 0x03
RegZLo   = 0x04
RegZHi   = 0x05

############## Cpntrol Register Value 
Mode_Standby    = 0b00000000
Mode_Continuous = 0b00000001
ODR_10Hz        = 0b00000000
ODR_50Hz        = 0b00000100
ODR_100Hz       = 0b00001000
ODR_200Hz       = 0b00001100
RNG_2G          = 0b00000000
RNG_8G          = 0b00010000
OSR_512         = 0b00000000
OSR_256         = 0b01000000
OSR_128         = 0b10000000
OSR_64          = 0b11000000
        
def signed_int(number) :
    #convert into binary and apply two's complement
    num = number
    binary_string = ""
    
    while number > 0:
        remainder = number % 2
        binary_string = str(remainder) + binary_string
        number = number//2
        
    leng = len(binary_string)

    if(leng<16):
        result = num
    else :
        result = num - 2**16
        
    return result

  
    
########### Init
ctrl1 = bytearray([Mode_Continuous|ODR_200Hz|RNG_8G|OSR_512])
i2c.writeto_mem(QMC5883L_ADDR,RegCTRL1, ctrl1)
i2c.writeto_mem(QMC5883L_ADDR,RegFBR, b'\x01')

while True :
    buffer = i2c.readfrom_mem(QMC5883L_ADDR,RegXLo,6)
         
    xLo = buffer[0]
    xHi = buffer[1] << 8
    yLo = buffer[2]
    yHi = buffer[3] <<8
    zLo = buffer[4]
    zHi = buffer[5] <<8

    x = xLo+xHi
    y = yLo+yHi
    z = zLo+zHi
    x_mag_bf = signed_int(x)/4096
    y_mag_bf = signed_int(y)/4096
    z_mag_bf = signed_int(z)/4096
    
    #print(f"x : {x_mag_bf},y : {y_mag_bf},z : {z_mag_bf}")
    print(x_mag_bf,y_mag_bf,z_mag_bf)
    sleep(1)
         