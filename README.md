# PID Motor Control Simulation (Discrete-Time)

## Overview

This project implements a discrete-time PID controller regulating a first-order motor model.

The simulation includes:

- Actuator saturation
- Measurement noise (Gaussian)
- Filtered derivative action
- Anti-windup (conditional integration)
- Step setpoint tracking

The goal is to study real-world PID behavior under non-ideal conditions.


---

## System Model

The motor is modeled as a first-order system:

dω/dt = (1/τ) * (K * u - ω)

Where:

- ω = motor speed
- u = control input (voltage)
- τ = motor time constant
- K = motor gain

The system is discretized using Euler integration.


---

## PID Controller

u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt

Where:

- e(t) = setpoint - measured_speed


### Implemented Features

1) Measurement Noise  
Gaussian noise is added to the measured speed.

2) Derivative Filtering  
To prevent noise amplification, the derivative term is filtered using:

D(s) = (Kd * s) / (1 + τd * s)

Discrete implementation:

d_k = d_k-1 + α (d_raw - d_k-1)

Where:

α = dt / (τd + dt)

This reduces high-frequency noise sensitivity.

3) Actuator Saturation  
Control voltage is limited to a fixed range:

u ∈ [min_voltage, max_voltage]

4) Anti-Windup (Conditional Integration)  
The integral term is updated only when the actuator is not saturated.


---

## Simulation Parameters

- dt = 0.01 s
- Total time = 10 s
- τ = 0.5
- K_motor = 1.0
- Kp = 10.0
- Ki = 2.0
- Kd = 3.0


---

## Output Plots

The simulation generates:

- Motor speed vs time
- Control signal vs time
- Measured speed (with noise) vs setpoint


---

## How to Run

1) Install dependencies:

pip install -r requirements.txt

2) Run the simulation:

python pid_simulation.py


---

## Engineering Insights

- Increasing Kp reduces rise time but increases overshoot.
- High Ki eliminates steady-state error but risks windup.
- Kd improves damping but amplifies measurement noise.
- Increasing τd reduces noise amplification but introduces phase lag.


---

## Future Improvements

- Back-calculation anti-windup
- Automatic PID tuning
- Second-order plant extension
- Frequency-domain analysis
- State-space implementation


---

## License

MIT License
