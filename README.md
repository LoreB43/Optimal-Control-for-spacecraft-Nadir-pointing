# Spacecraft Nadir Pointing – Optimal Control
Development of a simulation for satellite kinematics and dynamics using MATLAB/Simulink. Design and implementation of a Model-Based Optimal Controller to maintain a stable Nadir-pointing orientation (keeping the satellite's payload facing Earth).

<img src="https://github.com/user-attachments/assets/5855d8c0-a71d-4ffe-8860-229c7a5a1de8" width="35%"/>
<img src="https://github.com/user-attachments/assets/674ef203-c246-4900-aa4b-9d4b5a04e3e8" width="30%"/>

---

## Project Objectives

<img src="https://github.com/user-attachments/assets/3800d418-8f4f-4f13-bc6f-1c65ba855899" width="30%"/>

Many satellite missions (Earth observation, remote sensing, communication) require the spacecraft to keep a fixed orientation relative to Earth.

In this project we simulate a **12U CubeSat in Medium Earth Orbit (MEO)** and design a control strategy that:

- Performs an initial **slew maneuver** from a random orientation  
- Achieves **Nadir pointing**  
- Maintains stable pointing throughout the entire orbit  
- Respects **actuator torque limits**  
- Minimizes control effort
- **Gravity Gradient torque** and **Solar Radiation Pressure** are treated as system disturbances

The spacecraft attitude is represented using **quaternions**, avoiding singularities typical of Euler angles.  
The main control strategy involves the use of an optimal model-based controller like **MPC** and **FHOCP**

---

## Problems
A standard MPC strategy directly on the three rotation wheels.
- Very large number of decision variables  
- High computational cost  
- Not feasible for full orbital simulations
- Problem in real-time implementation

---

## Solutions

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

<img src="https://github.com/user-attachments/assets/9c753bad-53d2-4c18-a1f5-8c2c2441937a" width="50%"/>

---

## Performance Metrics and results
Different algorithms were tested:

- SQP (Sequential Quadratic Programming)
- Interior Point
- Active Set



- Iterations to convergence  
- Function evaluations  
- Computational time  
- Final cost value  

The **SQP algorithm** showed the best performance in terms of:

- convergence speed  
- computational efficiency  
- solution quality  

The optimized controller achieves:

- rapid convergence to the correct orientation  
- stable nadir pointing for the entire orbit  
- minimal control torque after alignment  
- compliance with actuator limits  

- Slew maneuver completed in **< 100 seconds**
- Pointing error converges to **zero**
- Control torque approaches **zero after stabilization**

This demonstrates the effectiveness of **parameterized optimal control using PID tuning**.

<img src="https://github.com/user-attachments/assets/f3038376-9f78-483b-aede-f29be43bcce0" width="20%"/>

---

## Future Work
Controllo per inizializzazioni piu ampie e messa in orbita del satellite

---

## How to run
