# Real-Time Attitude Estimation using Madgwick Filter (MATLAB)
Using phone sensor data (Accelerometer, Gyroscope and Magnetometer) to capture its attitude in real time using a Madgwick Filter. Sattelites use the same sensors to estimate their attitude, so it has been directly inspired by how Attitude estimation of satellites works in real life.

## Overview

This project implements a real-time 3D attitude estimation system using sensor data from a smartphone. The system estimates orientation using the Madgwick filter and visualizes it in MATLAB.

The implementation uses accelerometer, gyroscope, and magnetometer data to compute orientation in quaternion form and convert it into a rotation matrix for visualization.

Although a smartphone is used as the sensor source, the methodology is directly applicable to spacecraft attitude determination systems.

## Motivation

The project began with an attempt to estimate 3D orientation using a standard Kalman filter. However, this approach proved to be prone to drift and not well suited for nonlinear orientation estimation using quaternions.

This led to exploring alternative approaches, specifically the Madgwick filter, which is based on gradient descent optimization and designed for efficient real-time sensor fusion.

## System Architecture
1. Data is collected by the phone sensors using the Phyphox app (a custom experiment that collects all 3 sensor data is created)
2. The data is sent to the computer through Phyphox's inbuilt remote data feature
3. The browser receives .json data, which is extracted by using the webread() function in MATLAB
4. Data is passed through the Madgwick Filter which generates the new orientation based on the math inside.
5. The orientation is converted to a rotation matrix, which then visualised as 3D orientation inside MATLAB


## Sensor Data

The system uses:

1. Accelerometer for Gravity direction  
2. Gyroscope for Angular velocity  
3. Magnetometer for Heading reference  

These are the same core sensors used in satellite attitude estimation systems.

## Implementation Details
<img width="678" height="510" alt="image" src="https://github.com/user-attachments/assets/d98425b6-3988-4a8b-b8f3-d5a8e7cae658" />  


### 1. Quaternion Representation
Orientation is represented as a quaternion  
$q = [q_0, q_1, q_2, q_3]$  
with $q_0$ being the scalar part. Quaternions are used to avoid singularities and ensure smooth rotation.


### 2. Gradient Computation  
<img width="539" height="447" alt="image" src="https://github.com/user-attachments/assets/26627bb2-6001-4fab-989f-96f0f04ac61b" /> <img width="462" height="230" alt="image" src="https://github.com/user-attachments/assets/1ef481c0-4521-46b6-a42f-a129b93b2965" />   

<img width="504" height="175" alt="image" src="https://github.com/user-attachments/assets/5264864f-eb73-44d2-89f6-0379d6af3d8c" />

The final $f = [f_g, f_m]$ combines the error of the gravity sensor and magnetic field sensor  
Jacobian matrices are derived analytically and used to compute the gradient.  

Then the gradient is calculated as  
<img width="400" height="161" alt="image" src="https://github.com/user-attachments/assets/4ae4c9fa-56a0-4edf-90e3-5a52a1b84549" />  <img width="310" height="77" alt="image" src="https://github.com/user-attachments/assets/4dfb6760-30db-4a12-8e87-2318d0564119" />  
with $\nabla f = J^T f$

## Using Phyphox for live data

Steps:

1. Enable remote access in the app  
2. Use MATLAB webread() to fetch sensor values  
3. Parse JSON data and extract latest readings  

## Visualization

A custom 3D visualization is implemented in MATLAB, such that: 
- Screen faces top (Z axis)
- Top of the phone faces front (Y axis)
- Using right hand rule, phone's right is X axis
  
Orientation updated in real time

## Results
Stable real-time orientation estimation  
Accurate response to tilt and rotation  
Consistent alignment with physical motion  
Smooth visualization in MATLAB  

## How to Run
Install MATLAB  
Install Phyphox on your phone  
Enable remote access in the app  
Update the IP address in the MATLAB script  
Run the script  
Move your phone to see real-time orientation  

## References
- Madgwick, S. "Estimation of IMU and MARG orientation using a gradient descent algorithm"  
- Quaternion kinematics and rigid body rotation theory
