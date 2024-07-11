from mekf import MEKF
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
i2cline = I2C(id = 1, scl=Pin(19), sda = Pin(18),freq = 400000, timeout = 1000)
time.sleep(0.1)

print(i2cline.scan())
mpu_obj = MPU6050(1,18,19)  
mpu_obj.callibrate_gyro() #calibrate gyro 
mpu_obj.callibrate_acc()

# ax_bias = 0
# ay_bias = 0
# az_bias = 0
p_bias = 0
q_bias = 0
r_bias = 0
for i in range(100):
#     ax_bias += mpu_obj.read_acc()[0]
#     ay_bias += mpu_obj.read_acc()[1]
#     az_bias += mpu_obj.read_acc()[2]
    p_bias += mpu_obj.read_gyro()[0]
    q_bias += mpu_obj.read_gyro()[1]
    r_bias += mpu_obj.read_gyro()[2]
p_bias /= 100.0
q_bias /= 100.0
r_bias /= 100.0
# Carry out gyro_bias calculation and assign values to bx, by, bz
# Here I have assigned random values
bx = p_bias
by = q_bias
bz = r_bias

gravity = Vector(0, 0, -1)

# Magnetic field vector (some arbitrary fixed direction- north here)
magnetic_field = Vector(1.0, 0, 0).unit_vector

# Time step
dt = 0.1  # 10 ms
R_input = 0.0001*  Matrix.identity(6)
P_start = 0.0001 * Matrix.identity(6)
x_init = Vector(0, 0, 0, bx, by, bz) ## bx, by, bz are gyro biases; first calibrate and find biases at the start
std_dev_process = Vector(0.01, 0.01, 0.01, 0.001,0.001, 0.001)
inertial_acc_mag = Vector(gravity[0], gravity[1], gravity[2], magnetic_field[0], magnetic_field[1], magnetic_field[2])
mekf = MEKF(R_input, P_start, x_init, inertial_acc_mag, dt, std_dev_process)


while True:
    data = qmc(1,19,18)
    mx = data['x']
    my = data['y']
    mz = data['z']
    ax = mpu_obj.read_acc()[0]
    ay = mpu_obj.read_acc()[1]
    az = mpu_obj.read_acc()[2]
    gx = mpu_obj.read_gyro()[0]
    gy = mpu_obj.read_gyro()[1]
    gz = mpu_obj.read_gyro()[2]
    acc_meas = Vector(ax, ay, az)
    mag_meas = Vector(mx, my, mz)
    
    mekf.measurement_update(acc_meas, mag_meas)
    
    omega_meas = Vector(gx, gy, gz)
    mekf.predict_state(omega_meas)
    
    if Vector(mekf.del_x[0], mekf.del_x[1], mekf.del_x[2]).modulus >= 3.14/180 * 5:
        mekf.reset()
        
    print(f'{mekf.q_est.w}, {mekf.q_est.x}, {mekf.q_est.y}, {mekf.q_est.z}')

    time.sleep(0.01)