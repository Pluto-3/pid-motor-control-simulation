import numpy as np
import matplotlib.pyplot as plt

# Simulation Parameters
dt = 0.01          # time step (s)
T = 10             # total simulation time (s)
time = np.arange(0, T, dt)

# Motor Parameters
K_motor = 1.0      # motor gain
tau = 0.5          # motor time constant

# PID Gains
Kp = 10.0
Ki = 2.0
Kd = 3.0

# Derivative filter time constant
tau_d = 0.05

# Actuator Limits
min_voltage = -12
max_voltage = 12

# Noise
noise_sigma = 0.05

# Setpoint
setpoint = 10

# Initial States
omega = 0.0
error_integral = 0.0
prev_error = 0.0
prev_filtered_derivative = 0.0

# Storage
omega_list = []
control_list = []
measured_speed_list = []

# Simulation Loop
for t in time:

    # Measurement with noise
    measured_speed = omega + np.random.normal(0, noise_sigma)

    # Error
    error = setpoint - measured_speed

    # Raw derivative
    raw_derivative = (error - prev_error) / dt

    # Derivative filtering (low-pass)
    alpha = dt / (tau_d + dt)
    filtered_derivative = (
        prev_filtered_derivative
        + alpha * (raw_derivative - prev_filtered_derivative)
    )

    # Provisional control (before saturation)
    u_unsat = (
        Kp * error
        + Ki * error_integral
        + Kd * filtered_derivative
    )

    # Apply actuator saturation
    u = np.clip(u_unsat, min_voltage, max_voltage)

    # Anti-windup (conditional integration)
    if u == u_unsat:
        error_integral += error * dt

    # Motor update (Euler integration)
    omega += dt * (1 / tau) * (K_motor * u - omega)

    # Store results
    omega_list.append(omega)
    control_list.append(u)
    measured_speed_list.append(measured_speed)

    # Update states
    prev_error = error
    prev_filtered_derivative = filtered_derivative

plt.figure(figsize=(10, 4))
plt.plot(time, omega_list, label="Motor Speed")
plt.plot(time, [setpoint] * len(time), "--", label="Setpoint")
plt.xlabel("Time (s)")
plt.ylabel("Speed")
plt.title("PID Motor Speed Control")
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(10, 4))
plt.plot(time, control_list, label="Control Input (Voltage)")
plt.xlabel("Time (s)")
plt.ylabel("Control Signal")
plt.title("PID Control Effort")
plt.grid(True)
plt.show()

plt.figure(figsize=(10, 4))
plt.plot(time, measured_speed_list, label="Measured Speed (with noise)")
plt.plot(time, [setpoint] * len(time), "--", label="Setpoint")
plt.xlabel("Time (s)")
plt.ylabel("Measured Speed")
plt.title("Measured Speed with Noise")
plt.legend()
plt.grid(True)
plt.show()