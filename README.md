# pid-motor-control-simulation
Discrete-time PID control simulation of a first-order motor model with actuator saturation, measurement noise, derivative filtering, and anti-windup logic.

PID Motor Control Simulation (Discrete-Time)
Overview

This project implements a discrete-time PID controller regulating a first-order motor model. The simulation includes:

Actuator saturation

Measurement noise

Filtered derivative action

Anti-windup (conditional integration)

Step setpoint tracking

The objective is to study PID behavior under realistic non-ideal conditions.

System Model
Plant (First-Order Motor Model)

The motor is modeled as:

𝑑
𝜔
𝑑
𝑡
=
1
𝜏
(
𝐾
𝑢
−
𝜔
)
dt
dω
	​

=
τ
1
	​

(Ku−ω)

Where:

𝜔
ω = motor speed

𝑢
u = control input (voltage)

𝜏
τ = time constant

𝐾
K = motor gain

Discretized using Euler integration.

PID Controller
𝑢
(
𝑡
)
=
𝐾
𝑝
𝑒
(
𝑡
)
+
𝐾
𝑖
∫
𝑒
(
𝑡
)
𝑑
𝑡
+
𝐾
𝑑
𝑒
˙
(
𝑡
)
u(t)=K
p
	​

e(t)+K
i
	​

∫e(t)dt+K
d
	​

e
˙
(t)

Where:

𝑒
(
𝑡
)
=
𝑟
(
𝑡
)
−
𝑦
(
𝑡
)
e(t)=r(t)−y(t)

Features Implemented
1️⃣ Measurement Noise

Gaussian noise added to measured speed:

measured_speed = omega + np.random.normal(0, sigma)
2️⃣ Derivative Filtering

Pure derivative amplifies noise.
Implemented filtered derivative:

𝐷
(
𝑠
)
=
𝐾
𝑑
𝑠
1
+
𝜏
𝑑
𝑠
D(s)=
1+τ
d
	​

s
K
d
	​

s
	​


Discrete form:

𝑑
𝑘
=
𝑑
𝑘
−
1
+
𝛼
(
𝑑
𝑟
𝑎
𝑤
−
𝑑
𝑘
−
1
)
d
k
	​

=d
k−1
	​

+α(d
raw
	​

−d
k−1
	​

)

Where:

𝛼
=
𝑑
𝑡
𝜏
𝑑
+
𝑑
𝑡
α=
τ
d
	​

+dt
dt
	​


This reduces high-frequency noise amplification.

3️⃣ Actuator Saturation

Voltage constrained to:

u ∈ [min_voltage, max_voltage]
4️⃣ Anti-Windup (Conditional Integration)

Integral accumulates only if control signal is not saturated:

if u == u_unsat:
    error_integral += error * dt

Prevents excessive overshoot due to integral windup.

Simulation Parameters
Parameter	Value
dt	0.01 s
Total Time	10 s
τ	0.5
K_motor	1.0
Kp	10
Ki	2
Kd	3
Results

The simulation produces:

Motor speed vs time

Control input vs time

Measured speed (with noise) vs setpoint

Example behaviors observed:

Overshoot due to aggressive Kp and Ki

Noise amplification in derivative term (before filtering)

Anti-windup reducing post-saturation integral buildup

Tradeoff between derivative filtering and phase lag

How to Run
pip install -r requirements.txt
python pid_simulation.py
Engineering Insights

Increasing Kp reduces rise time but increases overshoot.

High Ki improves steady-state error but risks windup.

Kd improves damping but amplifies measurement noise.

Increasing τ_d reduces noise sensitivity but introduces phase lag.

Future Work

Implement back-calculation anti-windup

Add automatic PID tuning (Ziegler–Nichols / optimization-based)

Compare filtered vs unfiltered derivative

Extend plant to second-order model

Add frequency-domain analysis
