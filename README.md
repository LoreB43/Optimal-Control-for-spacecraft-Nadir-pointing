# Spacecraft Nadir Pointing – Optimal Control

Simulation and optimization of spacecraft **attitude control** to achieve and maintain **Nadir pointing** during an entire orbital period.

The project combines **orbital mechanics, spacecraft dynamics, control theory and nonlinear optimization**, implemented using **MATLAB and Simulink**.
<img width="700" height="525" alt="angle_PID" src="https://github.com/user-attachments/assets/f3038376-9f78-483b-aede-f29be43bcce0" />
<img width="742" height="716" alt="frames2" src="https://github.com/user-attachments/assets/3800d418-8f4f-4f13-bc6f-1c65ba855899" />
<img width="1500" height="502" alt="PID_flow" src="https://github.com/user-attachments/assets/9c753bad-53d2-4c18-a1f5-8c2c2441937a" />
<img width="850" height="736" alt="Reference-frames-ECI-body-LVLH-Target" src="https://github.com/user-attachments/assets/674ef203-c246-4900-aa4b-9d4b5a04e3e8" />

---

# Project Overview

Many satellite missions (Earth observation, remote sensing, communication) require the spacecraft to keep a fixed orientation relative to Earth.

In this project we simulate a **12U CubeSat in Medium Earth Orbit (MEO)** and design a control strategy that:

- Performs an initial **slew maneuver** from a random orientation  
- Achieves **Nadir pointing**  
- Maintains stable pointing throughout the entire orbit  
- Respects **actuator torque limits**  
- Minimizes control effort  

The spacecraft attitude is represented using **quaternions**, avoiding singularities typical of Euler angles.

---

# System Model

The simulation includes several components.

## Orbital Dynamics

- Near-polar **MEO orbit**
- Two-body gravitational model with **J2 perturbation**

## Spacecraft Dynamics

- Rigid body dynamics using **Euler equations**
- Inertia matrix derived from CubeSat geometry

## Disturbances

- **Gravity Gradient torque**
- **Solar Radiation Pressure**

## Actuation

- Three **reaction wheels**
- Torque saturation limits

The spacecraft attitude is propagated using **quaternion kinematics and angular velocity dynamics**.

---

# Control Strategy

Two optimal control formulations were investigated.

## 1. Torque-Based Optimal Control (FHOCP)

The control input is the **reaction wheel torque sequence** optimized over a finite horizon.

### Objective

- Minimize pointing error during the maneuver

### Limitations

- Very large number of decision variables  
- High computational cost  
- Not feasible for full orbital simulations  

---

## 2. PID Parameter Optimization

To reduce complexity, the control law is parameterized using a **PID controller**.

Control law:

```
M = -kp*q_error - kd*ω_error - ki ∫q_error dt
```

Instead of optimizing torque directly, the solver optimizes:

```
PID = [kp, ki, kd]
```

This reduces the optimization problem to **only three decision variables**.

The parameters are tuned using **nonlinear constrained optimization**.

---

# Optimization

The PID gains are optimized using **MATLAB `fmincon`**.

Different algorithms were tested:

- SQP (Sequential Quadratic Programming)
- Interior Point
- Active Set

## Performance Metrics

- Iterations to convergence  
- Function evaluations  
- Computational time  
- Final cost value  

### Result

The **SQP algorithm** showed the best performance in terms of:

- convergence speed  
- computational efficiency  
- solution quality  

---

# Simulation Results

The optimized controller achieves:

- rapid convergence to the correct orientation  
- stable nadir pointing for the entire orbit  
- minimal control torque after alignment  
- compliance with actuator limits  

### Main Outcomes

- Slew maneuver completed in **< 100 seconds**
- Pointing error converges to **zero**
- Control torque approaches **zero after stabilization**

This demonstrates the effectiveness of **parameterized optimal control using PID tuning**.

---

# Technologies Used

- MATLAB
- Simulink
- Nonlinear Optimization (`fmincon`)
- Control Systems
- Orbital Mechanics
- Quaternion-based Attitude Dynamics

---

# Repository Structure

Example structure:

```
.
├── Simulink_model
├── MATLAB_scripts
│   ├── optimization.m
│   ├── attitude_dynamics.m
│   └── cost_function.m
├── results
├── figures
└── report
```

---

# Key Concepts Demonstrated

- Spacecraft attitude dynamics
- Quaternion kinematics
- Optimal control
- Nonlinear constrained optimization
- Control system design
- Numerical solver comparison
