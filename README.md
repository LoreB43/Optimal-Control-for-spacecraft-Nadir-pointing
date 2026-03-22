# Spacecraft Nadir Pointing – Optimal Control
Development of a simulation for satellite kinematics and dynamics using MATLAB/Simulink. Design and implementation of a Model-Based Optimal Controller to maintain a stable Nadir-pointing orientation (keeping the satellite's payload facing Earth).

<img src="https://github.com/user-attachments/assets/5855d8c0-a71d-4ffe-8860-229c7a5a1de8" width="35%"/>
<img src="https://github.com/user-attachments/assets/674ef203-c246-4900-aa4b-9d4b5a04e3e8" width="30%"/>

For full details look at the report: https://github.com/LoreB43/Optimal-Control-for-spacecraft-Nadir-pointing/blob/main/Report.pdf

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
A standard MPC strategy directly on the three rotation wheels leads to different problems:
- Very large number of decision variables (three torques times the number of istants in the finite horizon of the prediction)  
- High computational cost  
- Not feasible for full orbital simulations
- Problem in real-time implementation due to complex model and long time to find an optimal solution

---

## Solutions
To solve this problem, many solutions are evaluated:
- To reduce complexity, the control law is parameterized using a **PID controller** to be able to keep the decision varible indipendent respect to the horizon of the model prediction
- Compile in C some code portions
- Evaluate different algorithm to find solutions effectively based on the complexity of the model. We compared SQP (Sequential Quadratic Programming), Interior Point, Active Set

This reduces the optimization problem to **only three decision variables** using the PID parametrization.  
<img src="https://github.com/user-attachments/assets/9c753bad-53d2-4c18-a1f5-8c2c2441937a" width="50%"/>

C programming language being a compiled language rather than an interpreted one is faster 

The **SQP algorithm** showed the best performance in terms of convergence speed, computational efficiency, solution quality.

<img src="https://github.com/user-attachments/assets/14e8f7bd-a77e-41d5-8f4a-d110264a097c" width="50%"/>

---

## Results

The optimized controller achieves:

- rapid convergence to the correct orientation. Slew maneuver completed in **< 100 seconds** 
- stable nadir pointing for the entire orbit. Pointing error converges to **zero**
- minimal control torque after alignment, Control torque approaches **zero after stabilization**  
- compliance with actuator limits  

This demonstrates the effectiveness of **parameterized optimal control using PID tuning**.

Here are the pointing error before and after the controller implementation:

<img src="https://github.com/user-attachments/assets/41e6fa6e-5d51-4df5-9333-a4ef942598f0" width="30%" />

<img src="https://github.com/user-attachments/assets/f3038376-9f78-483b-aede-f29be43bcce0" width="30%"/>

---

## Future Work
Design the controller to handle larger initial attitude errors and high angular velocities. Implement a Detumbling Mode (B-Cross or equivalent) to stabilize the satellite from its high-energy post-separation state, reducing kinetic energy before transitioning to the fine Nadir-pointing mode.

---

## How to run

- Requires Matlab/Simulink at least 2023 version
- Run main.m to obtain results (change dynamics_2024 to dynamics_2023, which constains the model dynamics computation on which model prediction is based, according to your matlab version)
