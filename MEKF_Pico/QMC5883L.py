from machine import I2C, Pin
import time

#calibration parameters
b1 = 0.073946
b2 = 0.067766
b3 = -0.206394
x1 = 1.083645
x2 = 0.044877
x3 = -0.035526
y1 = 0.044877
y2 = 1.253382
y3 = 0.012988
z1 = -0.035526
z2 = 0.012988
z3 = 1.163102

def calibration(x_mag,y_mag, z_mag):
    x_fin = (x_mag - b1) * x1 + (y_mag - b2) * x2 + (z_mag - b3) * x3
    y_fin = (x_mag - b1) * y1 + (y_mag - b2) * y2 + (z_mag - b3) * y3
    z_fin = (x_mag - b1) * z1 + (y_mag - b2) * z2 + (z_mag - b3) * z3
    return x_fin,y_fin,z_fin

##Register location
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

QMC5883L_ADDR = 0x0D
#for two's complement
def signed_int(number) :
    if(number>32768) :
        number = number - 65536
    else :
        number
    return number

def qmc(iid, SCL, SDA) :
    
    ctrl1 = bytearray([Mode_Continuous|ODR_200Hz|RNG_8G|OSR_512])
    pinSDA = Pin(SDA)
    pinSCL = Pin(SCL)
    i2c = I2C(iid ,freq=2000000, scl=pinSCL, sda=pinSDA)
    time.sleep(0.0001)
    i2c.writeto_mem(QMC5883L_ADDR,RegCTRL1, ctrl1)
    i2c.writeto_mem(QMC5883L_ADDR,RegFBR, b'\x01')
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
    x_mag = signed_int(x)/4096
    y_mag = signed_int(y)/4096
    z_mag = signed_int(z)/4096
    x_fin,y_fin,z_fin = calibration(x_mag,y_mag,z_mag)
    
    return { 'x' : x_fin, 'y' : y_fin, 'z': z_fin}