# Tilt Stabilization System

## 📌 Project Overview  
This project was developed as part of the **EE-361 Principles of Feedback Control Lab** course during Fall 2019. The aim was to demonstrate **angular position control** of a surface along two Cartesian axes. The system rejects disturbances such as additional weight on the controlled surface and changes in the orientation of its mounting structure. The tilt stabilization technique can have applications ranging from **gimbal-based camera stabilization** to **aeronautical equipment**.

**REPORT: https://github.com/SarwanShah/HU_2019_Tilt-Stablization-Control-System/blob/main/Final-Report.pdf**

**VIDEO DEMONSTRATION:**

<a href="https://www.youtube.com/watch?v=KZVrodOSyYc" target="_blank">
    <img src="https://img.youtube.com/vi/KZVrodOSyYc/maxresdefault.jpg" alt="Watch the video" width="400">
</a>


## 🛠 Project Features  
- **Dual-Axis Control**: Independent control systems for both X and Y axes.
- **Disturbance Rejection**: The system rejects disturbances introduced in various orientations.
- **MATLAB Model Simulation**: Validation of system response through **transfer function modeling**.
- **Real-Time System Performance**: Verification and visualization of system behavior with actual disturbances.
- **Feedback Sensor Integration**: Utilizes the **MPU6050** gyroscopic sensor for continuous angle feedback.

## 🏗 Implementation Details  

### ➤ **System Design**  
- A **square-shaped structure** with two internal frames (inner and outer) allows free rotation along the X and Y axes.
- Bearings minimize friction, and **24V DC motors** enable position control.
  
### ➤ **Transfer Functions**  
- **Motor Transfer Function**: Obtained via **black-box modeling** and validated using MATLAB.
- **Plant Transfer Function**: Derived from the system's physical parameters, accounting for inertia but neglecting friction and damping due to balanced pivot and bearings.
  
### ➤ **Control System**  
- The system uses a **proportional controller** to generate motor actuation signals based on angular error values from the gyroscopic sensor.
  
### ➤ **Feedback and Sensing**  
- **MPU6050 gyroscopic sensor**: Provides real-time feedback for error correction.
- **Complementary Filter**: Combines accelerometer and gyroscopic data to minimize drift and steady-state errors.

## ⚙ Components Used  
| Component              | Description                           |
|------------------------|---------------------------------------|
| **Arduino**            | Microcontroller for control logic     |
| **MPU6050**            | Gyroscopic sensor for feedback        |
| **L298 Motor Driver**  | Amplifies control signals to motors   |
| **24V DC Motors**      | Provides rotational motion            |
| **Bearings**           | Reduces friction for smoother movement|

## 🧪 Results and Validation  

### Experimental Model Simulation  
The system model was simulated in MATLAB to validate its response to angular disturbances. Key response parameters include:
- **Rise Time**: 2.4884 seconds (X-axis) / 0.2242 seconds (Y-axis)
- **Settling Time**: 4.4310 seconds (X-axis) / 0.3992 seconds (Y-axis)

### Real System Simulation  
The system's real-time response was captured and plotted. It successfully rejected angular disturbances with:
- **Settling Time**: Approximately 2-4 seconds  
- **Steady-State Accuracy**: Minimal steady-state error along the X-axis, with minor discrepancies along the Y-axis.

## 📥 How to Use  
1. Assemble the system as per the provided design.
2. Upload the **Arduino code** (included in the repository) to the microcontroller.
3. Apply angular disturbances to test system stability.
4. Observe and record system responses using **MATLAB** or other tools.

---

This project provided hands-on experience in **control systems, sensor integration, and proportional control design**. It successfully met its objectives and demonstrated practical applications of angular stabilization techniques.
