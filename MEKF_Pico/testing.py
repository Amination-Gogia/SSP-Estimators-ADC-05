import numpy as np
import matplotlib.pyplot as plt
from mekf_software import MEKF
from matrix import Matrix
from vector import Vector
from quaternion import Quaternion

# Gravity vector (pointing downward along the z-axis)
gravity = Vector(0, 0, -1)

# Magnetic field vector (some arbitrary fixed direction)
magnetic_field = Vector(0.5, 0.5, 0.3).unit_vector
magnetic_field_foolish = magnetic_field ## Vector(1, 0, 0).unit_vector
# Time step
dt = 0.1  # 10 ms

# Number of time steps
num_steps = 1000

# Initial quaternion (identity, no rotation)
initial_quaternion = Quaternion(1, 0, 1, 0)
initial_quaternion.normalize()

# Generate random angular velocity measurements (omega_meas)
np.random.seed(42)  # For reproducibility

bx, by, bz = 0.1, 0.08, 0.09
omega_x, omega_y, omega_z = 0.5, 0.4, 0.3

alpha_x, alpha_y, alpha_z = 0.2, 0.1, 0.3
alpha = Vector(alpha_x, alpha_y, alpha_z)

omega_actual = [Vector(omega_x, omega_y, omega_z) for i in range(num_steps)]
omega_meas = [ Vector(bx +omega_x + np.random.uniform(-0.001, 0.001), 
                     by + omega_y + np.random.uniform(-0.001, 0.001), 
                     bz + omega_z + np.random.uniform(-0.001, 0.001)) for _ in range(num_steps)]

# Generate ground truth quaternions
true_quaternions = [initial_quaternion]
for omega in omega_actual:
    # q = true_quaternions[-1]
    
    # q_next = q + q.epsilon() * 0.5 * (omega)* dt
    # q_next.normalize()
    # true_quaternions.append(q_next)
    q = true_quaternions[-1]
    dq = Quaternion(0, omega[0], omega[1], omega[2]) * q * (0.5 * dt)
    q_next = q + dq
    q_next.normalize()
    true_quaternions.append(q_next)

# Convert to numpy array for easier plotting
true_quat_array = np.array([[q.w, q.x, q.y, q.z] for q in true_quaternions])

# Initialize MEKF
R_input = 0.0001*  Matrix.identity(6)
P_start = 0.0001 * Matrix.identity(6)
x_init = Vector(0, 0, 0, bx , by, bz)
std_dev_process = Vector(0.01, 0.01, 0.01, 0.001,0.001, 0.001)
inertial_acc_mag = Vector(gravity[0], gravity[1], gravity[2], magnetic_field_foolish[0], magnetic_field_foolish[1], magnetic_field_foolish[2])

mekf = MEKF(R_input, P_start, x_init, inertial_acc_mag, dt, std_dev_process)

# Measurements (simulated as slightly noisy versions of true values)
acc_measurements = [Quaternion(*true_quat_array[_]).attitude_matrix() * gravity + Vector(np.random.normal(0.1, 0.01), np.random.normal(-0.1, 0.1), np.random.normal(-0.1, 0.1)) for _ in range(num_steps)]
mag_measurements = [Quaternion(*true_quat_array[_]).attitude_matrix() * magnetic_field + Vector(np.random.normal(0.1, 0.01), np.random.normal(-0.1, 0.1), np.random.normal(-0.1, 0.1)) for _ in range(num_steps)]

# Run MEKF over the generated data
estimated_quaternions = [initial_quaternion]
for i in range(num_steps):
    mekf.predict_state(omega_meas[i])
    mekf.measurement_update(acc_measurements[i], mag_measurements[i])
    estimated_quaternions.append(mekf.q_est)
    if Vector(mekf.del_x[0], mekf.del_x[1], mekf.del_x[2]).modulus >= 3.14/180 * 5:
        mekf.reset()

# Convert to numpy array for easier plotting
estimated_quat_array = np.array([[q.w, q.x, q.y, q.z] for q in estimated_quaternions])

# Plotting the quaternion components
time_steps = np.arange(num_steps + 1) * dt
fig, axs = plt.subplots(4, 1, figsize=(10, 8), sharex=True)

for i, component in enumerate(['w', 'x', 'y', 'z']):
    axs[i].plot(time_steps, true_quat_array[:, i], label='True', color='blue')
    axs[i].plot(time_steps, estimated_quat_array[:, i], label='Estimated', color='red', linestyle='dashed')
    axs[i].set_ylabel(f'Quaternion {component}')
    axs[i].set_ylim(-1.1, 1.1)  # Set y-axis limits from -1.1 to 1.1
    axs[i].legend()

axs[-1].set_xlabel('Time (s)')
plt.suptitle('Quaternion Components: True vs Estimated')
plt.tight_layout()
plt.show()
