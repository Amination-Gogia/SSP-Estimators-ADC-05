# The `mekf` class written in `Python` to be used on Raspberry Pi Pico

## Contents
- Libraries implementing the classes `Matrix`, `Vector` and `Quaternion` with the necessary methods for the code to run
- The `MEKF` class that determines attitude dynamically
  - Prediction using gyro measurements
  - Measurement update using accelerometer readings and magnetic field vectors

## Guidelines to Use
- Write a `main.py`, importing the modules `matrix`, `vector`,`quaternion` and `mekf`
- `main.py` should take be able to access sensor data, and contain an instance of the `MEKF` class
- Initialise the mekf variable with appropriate noise parameters, and the gyro_bias in the initial state vector
- The sensor measurements should be correctly passed in to the mekf methods
- While running on pico, upload `main`, `matrix`, `vector`,`quaternion` and `mekf`, and run `main`
- `testing.py` contains a simulation example using the `MEKF` class; the placeholder data can be changed and passed to check results
- `testing.py` should also be run with `matrix`, `vector`,`quaternion` and `mekf` in the same directory or with their locations added to the `sys.path`
