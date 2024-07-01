from matrix import Matrix
from vector import Vector
from quaternion import Quaternion
from mekf import MEKF
import time ## Here you might need to do utime

# Carry out gyro_bias calculation and assign values to bx, by, bz
# Here I have assigned random values
bx = 0.03
by = 0.01
bz = 0.02

gravity = Vector(0, 0, -1)

# Magnetic field vector (some arbitrary fixed direction)
magnetic_field = Vector(0.5, 0, 0.5).unit_vector

# Time step
dt = 0.1  # 10 ms
R_input = 0.0001*  Matrix.identity(6)
P_start = 0.0001 * Matrix.identity(6)
x_init = Vector(0, 0, 0, bx, by, bz) ## bx, by, bz are gyro biases; first calibrate and find biases at the start
std_dev_process = Vector(0.01, 0.01, 0.01, 0.001,0.001, 0.001)
inertial_acc_mag = Vector(gravity[0], gravity[1], gravity[2], magnetic_field[0], magnetic_field[1], magnetic_field[2])
mekf = MEKF(R_input, P_start, x_init, inertial_acc_mag, dt, std_dev_process)

while True:
    magX, magY, magZ = data_from_magnetometerX, data_from_magnetometerY, data_from_magnetometerZ #(Calibrated but unnormalized)
    accX, accY, accZ = data_from_accelerometerX, data_from_accelerometerY, data_from_accelerometerZ
    
    acc_meas = Vector(accX, accY, accZ)
    mag_meas = Vector(magX, magY, magZ)

    mekf.measurement_update(acc_meas, mag_meas)

    wx, wy, wz = raw_data_from_gyro_x, raw_data_from_gyro_y, raw_data_from_gyro_z ##(gyro bias NOT subtracted)
    omega_meas = Vector(wx, wy, wz)

    mekf.predict_state(omega_meas)

    if Vector(mekf.del_x[0], mekf.del_x[1], mekf.del_x[2]).modulus >= 3.14/180 * 5:
        mekf.reset()

    print(f'{mekf.q_est.w}, {mekf.q_est.x}, {mekf.q_est.y}, {mekf.q_est.z}')

    time.sleep(0.1) ## On pico, utime.sleep() might work instead
