# Kalman Filtering Codebase for ADC-05

## Contents

### Matlab Simulations
- **[adc02revised folder](./Matlab%20Simulations/adc02revised)**: 
  - MATLAB Classes 'MEKF' and 'MEKF_lvlh' for Multiplicative Extended Kalman Filtering

### Matrix Lib
- **Python Libraries**:
  - Implementations of [Matrix](./Matrix%20Lib/matrix.py), [Vector](./Matrix%20Lib/vector.py), and [Quaternion](./Matrix%20Lib/quaternion.py) classes

### Thonny Kryptonite
- **[qmc_pico_interfaced.py](./Thonny%20Kryptonite/qmc_pico_interfaced.py)**: 
  - Python program to access QMC5883L magnetometer readings on Raspberry Pi Pico

### mekf3_mag
- **[mekf3_mag.ino](./mekf3_mag/mekf3_mag/mekf3_mag.ino)**: 
  - Arduino program implementing Multiplicative Extended Kalman Filter (MEKF) using MPU6050 (Inertial Measurement Unit) and QMC5883L
