# CubeSat-Simulation
Physics-based CubeSat simulation integrating two-body orbital propagation, quaternion attitude dynamics, PD control, atmospheric drag, and stochastic sensor modeling. Validates orbital conservation and drag-induced decay while analyzing attitude stability and measurement error statistics using numerical integration and visualization.

# CubeSat Orbit, Attitude, and Sensor Simulation Framework

---

## Introduction

This project presents a physics-based simulation framework for modeling the translational and rotational dynamics of a CubeSat operating in Low Earth Orbit (LEO). The simulation integrates orbital mechanics, rigid-body attitude dynamics, closed-loop control, atmospheric drag modeling, and stochastic sensor behavior within a unified numerical environment.

The objective of this work was to develop a computationally consistent and scientifically grounded spacecraft simulation suitable for graduate-level study and engineering analysis. The framework demonstrates fundamental concepts in orbital dynamics, attitude determination and control systems (ADCS), numerical integration, and statistical error modeling.

Such integrated simulation environments are essential in spacecraft design, mission analysis, and Guidance, Navigation, and Control (GNC) development.

---

# Aim of the Project

The primary objectives of this project were:

- To implement a two-body orbital propagation model using numerical integration.
- To model spacecraft attitude dynamics using quaternion-based rigid-body kinematics.
- To design and simulate a proportional-derivative (PD) attitude control system.
- To incorporate atmospheric drag and analyze orbital decay behavior.
- To simulate realistic GPS and gyroscope measurement noise.
- To validate simulation outputs through statistical and physical consistency checks.
- To visualize dynamic spacecraft behavior using multi-dimensional plots.

---

# Problems Addressed

This project addresses several core engineering challenges encountered in spacecraft dynamics and control:

### 1. Orbital Propagation Accuracy
- Stable numerical integration of nonlinear gravitational dynamics.
- Conservation of orbital elements under conservative forces.
- Sensitivity to non-conservative perturbations (drag).

### 2. Atmospheric Drag Modeling
- Exponential atmospheric density approximation.
- Ballistic coefficient influence on orbital decay.
- Semi-major axis variation under dissipative forces.

### 3. Attitude Representation and Stability
- Use of quaternions to avoid Euler angle singularities.
- Coupled rotational dynamics modeling.
- Numerical stability in attitude propagation.

### 4. Closed-Loop Control Design
- Reference attitude generation (nadir-pointing).
- PD torque control implementation.
- Actuator saturation constraints.

### 5. Sensor Noise and Uncertainty
- Gaussian position measurement noise.
- Gyroscope white noise and bias random walk.
- Statistical validation of measurement error.

---

# Methodology

The simulation framework follows a structured physics-based modeling approach.

---

## 1. Orbital Dynamics Model

The spacecraft translational motion is governed by Newton’s law of gravitation:

    r̈ = -μ r / |r|³

Where:
- μ is Earth’s gravitational parameter.
- r is the inertial position vector.

### Numerical Integration

The equations of motion are integrated using the Fourth-Order Runge–Kutta (RK4) method with a 1-second time step over multi-hour simulation intervals.

### Atmospheric Drag Extension

An optional drag acceleration term is included:

    a_d = -1/2 ρ Cd (A/m) |v| v

Where:
- ρ is modeled using an exponential atmosphere.
- Cd is the drag coefficient.
- A is cross-sectional area.
- m is spacecraft mass.

This enables examination of non-conservative orbital behavior.

---

## 2. Keplerian Element Conversion

The framework includes:

- Conversion from classical Keplerian elements to inertial Cartesian state vectors.
- Back-conversion from state vectors to orbital elements.
- Verification of conservation properties.

### Observed Behavior

Without drag:
- Semi-major axis remains constant.
- Eccentricity remains stable.
- Inclination remains constant.
- Numerical deviations are limited to floating-point precision.

With drag enabled:
- Semi-major axis decreases measurably.
- Eccentricity exhibits slight variation.
- Inclination remains approximately constant.

---

## 3. Attitude Dynamics

Spacecraft attitude is represented using quaternions:

    q = [qx, qy, qz, qw]

Quaternion kinematics are given by:

    q̇ = 1/2 Ω(ω) q

Rigid-body rotational dynamics follow:

    J ω̇ + ω × (J ω) = τ

Where:
- J is the spacecraft inertia tensor.
- ω is the angular velocity in the body frame.
- τ is applied control torque.

The same RK4 integration method is used for rotational states.

---

## 4. Attitude Control System

A nadir-pointing reference attitude is constructed such that:

- The body +Z axis points toward the Earth center.
- The body +X axis aligns approximately with the velocity direction.

A proportional-derivative (PD) control law is applied:

    τ = -Kp e - Kd ω

Where:
- e is derived from quaternion error.
- Kp and Kd are controller gains.

Torque saturation limits are imposed to reflect actuator constraints.

Simulation results show convergence of angular rates toward zero and reduction of attitude error over time.

---

## 5. Sensor Modeling

### GPS Position Model

The measurement model is:

    r_meas = r_true + n

Where:
- n is zero-mean Gaussian noise with standard deviation σ per axis.

For σ = 8 m, observed results include:

- Mean position error ≈ 12.79 m
- RMSE ≈ 13.87 m
- 95th percentile ≈ 22 m

These values are consistent with the theoretical expectation for 3D Gaussian noise:

    E[||n||] ≈ 1.596 σ

### Gyroscope Model

The gyroscope model includes:

- White Gaussian measurement noise.
- Bias random walk evolution:

    b_{k+1} = b_k + σ_rw √dt w_k

This reflects realistic MEMS sensor drift behavior.

---

# Simulation Outputs

The framework generates the following visual analyses:

- 3D orbit trajectory
- Altitude versus time
- Semi-major axis versus time (drag case)
- Attitude error versus time
- Angular rate versus time
- Position error histogram

These outputs support physical validation and statistical evaluation.

---

# Tools Used

- Python
- NumPy
- Matplotlib
- RK4 numerical integration
- Quaternion algebra

Conceptual foundations include:

- Orbital mechanics
- Rigid-body dynamics
- Control systems engineering
- Stochastic processes
- Statistical error analysis

---

# Analysis

### Conservative Case (No Drag)

- Orbital elements remain constant.
- Attitude control stabilizes angular velocity.
- Measurement error distribution matches theoretical predictions.

### Non-Conservative Case (Drag Enabled)

- Semi-major axis decreases over time.
- Slight eccentricity variation observed.
- System remains numerically stable.

These results confirm both physical consistency and numerical reliability.

---

# Findings

1. The two-body model conserves orbital elements as expected.
2. Atmospheric drag produces measurable orbital decay.
3. Quaternion-based propagation ensures numerical robustness.
4. PD control effectively stabilizes spacecraft attitude.
5. Sensor error statistics align with theoretical Gaussian expectations.
6. RK4 integration provides stable long-duration performance.

---

# Conclusion

This project demonstrates the development of a comprehensive CubeSat simulation framework integrating orbital dynamics, attitude dynamics, control systems, and sensor uncertainty modeling.

The simulation validates theoretical expectations in both conservative and non-conservative regimes and provides structured visualization of spacecraft behavior.

The framework is suitable as a foundation for further research in spacecraft dynamics, GNC development, and autonomous spacecraft simulation environments.

---

# Future Work

Potential extensions include:

- Extended Kalman Filter for orbit estimation.
- Gravity-gradient disturbance torque modeling.
- Reaction wheel dynamics.
- Monte Carlo sensitivity analysis.
- Earth rotation and ECEF frame transformation.
- Higher-fidelity atmospheric models.
- Event-based pointing performance metrics.

---

# Academic Context

This project reflects a structured integration of theoretical mechanics, numerical analysis, and control systems engineering. It demonstrates the ability to translate physical models into stable computational implementations and to validate results using statistical reasoning.

The work provides a foundation for advanced study in aerospace engineering, spacecraft systems, and computational physics.
