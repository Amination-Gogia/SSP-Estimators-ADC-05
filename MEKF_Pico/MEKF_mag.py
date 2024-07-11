from mekf_hardware import MEKF
from matrix import Matrix
from vector import Vector
from quaternion import Quaternion
from MPU import MPU6050
from QMC5883L import qmc
import math
import time
from machine import I2C, Pin, PWM, UART

#tx = Pin(16,Pin.OUT, value = 0)
#rx = Pin(17, Pin.OUT, value = 1)
#time.sleep(1)
#uart = UART(0,57600, bits=8, parity=None, stop=1, rx = Pin(17), tx = Pin(16))
i2cline = I2C(id = 1, scl=Pin(7), sda = Pin(6),freq = 400000, timeout = 1000)
time.sleep(0.1)

#print(i2cline.scan())
mpu_obj = MPU6050(1,6,7)  
#mpu_obj.callibrate_gyro() #calibrate gyro 
mpu_obj.callibrate_acc()

# ax_bias = 0
# ay_bias = 0
# az_bias = 0
p_bias = 0
q_bias = 0
r_bias = 0

b_x_0 = 0
b_y_0 = 0
b_z_0 = 0

ax0, ay0, az0 = 0, 0, 0

for i in range(10):
#     ax_bias += mpu_obj.read_acc()[0]
#     ay_bias += mpu_obj.read_acc()[1]
#     az_bias += mpu_obj.read_acc()[2]
    p_bias += mpu_obj.read_gyro()[1]
    q_bias += mpu_obj.read_gyro()[0]
    r_bias += -mpu_obj.read_gyro()[2]
    #print(i)
    data = qmc(1, 7, 6)
    b_x_0 += - data['x']
    b_y_0 += data['y']
    b_z_0 += - data['z']
    
    ax0 += mpu_obj.read_acc()[1]
    ay0 += mpu_obj.read_acc()[0]
    az0 += -mpu_obj.read_acc()[2]
    
p_bias /= 10.0
q_bias /= 10.0
r_bias /= 10.0

b_x_0 /= 10
b_y_0 /= 10
b_z_0 /= 10

# Carry out gyro_bias calculation and assign values to bx, by, bz
# Here I have assigned random values
bx = p_bias * (3.14/180)
by = q_bias * (3.14/180)
bz = r_bias * (3.14/180)

gravity = Vector(ax0, ay0, az0)

# Magnetic field vector (some arbitrary fixed direction- north here)
magnetic_field = Vector(b_x_0, b_y_0, b_z_0).unit_vector

# Time step
dt = 0.1  # 10 ms
R_input = 0.0001*  Matrix.identity(6)
for i in range(3):
    R_input[i][i] = (0.02/9.8) ** 2
    R_input[3 + i][3 + i] = (5/230) ** 2
P_start = 0.0001 * Matrix.identity(6)
x_init = Vector(0, 0, 0, bx, by, bz) ## bx, by, bz are gyro biases; first calibrate and find biases at the start
std_dev_process = Vector(0.01, 0.01, 0.01, 0.001,0.001, 0.001)
inertial_acc_mag = Vector(gravity[0], gravity[1], gravity[2], magnetic_field[0], magnetic_field[1], magnetic_field[2])
mekf = MEKF(R_input, P_start, x_init, inertial_acc_mag, dt, std_dev_process)


while True:
    data = qmc(1,7,6)
    mx = -data['x']
    my = data['y']
    mz = -data['z']
    ax = mpu_obj.read_acc()[1]
    ay = mpu_obj.read_acc()[0]
    az = -mpu_obj.read_acc()[2]
    gx = mpu_obj.read_gyro()[1]
    gy = mpu_obj.read_gyro()[0]
    gz = -mpu_obj.read_gyro()[2]
    acc_meas = Vector(ax, ay, az)
    mag_meas = Vector(mx, my, mz)
    
    mekf.measurement_update(acc_meas, mag_meas)
    
    omega_meas = Vector(gx, gy, gz) * (3.14/180)
    mekf.predict_state(omega_meas)
    
    if Vector(mekf.del_x[0], mekf.del_x[1], mekf.del_x[2]).modulus >= 3.14/180 * 5:
        mekf.reset()
        
    print(f'{mekf.q_est.w}, {mekf.q_est.x}, {mekf.q_est.y}, {mekf.q_est.z}')

    time.sleep(dt)