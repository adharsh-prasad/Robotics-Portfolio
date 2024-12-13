# **Trajectory Tracking Control of an Anthropomorphic Robotic Arm**

<div align="center"> <img src="Plots/Robotic Arm Simulation.gif" width="300"/> <p align="center"> <em>3-DOF Robotic Arm Trajectory Tracking Simulation</em> </p> </div>

This repository contains the code and documentation for the **Trajectory Tracking Control of an Anthropomorphic Robotic Arm** project. The project showcases the integration of control theory, specifically a Proportional-Derivative (PD) controller, to achieve accurate trajectory tracking for a robotic arm. The simulation visualizes the arm's performance in tracking complex trajectories under varying conditions.

---

## **Motivation**

This project represents a significant milestone in my academic and personal journey. As a graduate student at Arizona State University (ASU), I sought to bridge my background in control systems with my interest in robotics. This project provided the perfect opportunity to merge these domains.

Notably, it was the only project in the class to fully integrate control theory, kinematic modeling, and dynamic simulation. The result was a peer-reviewed project that earned first place and opened doors for further research opportunities at the Space Robotics Lab.

---

## **Features**

- **Custom Kinematics**:
  - Forward kinematics using Denavit-Hartenberg (D-H) parameters.
  - Geometrically solved inverse kinematics for accurate joint configuration.

- **Trajectory Control**:
  - Designed and implemented a PD controller for precise trajectory tracking.
  - Tuned controller gains for zero steady-state error and minimal overshoot.

- **Simulation and Visualization**:
  - Comprehensive 3D simulation of trajectory tracking.
  - Robust performance demonstrated with disturbance torque simulations.

- **Optimization**:
  - Comparison between traditional tuning methods and Particle Swarm Optimization (PSO) for control parameter tuning.

---

## **Technical Approach**

### **Kinematic Model**
Forward and inverse kinematics were developed to enable precise motion planning:
- **Forward Kinematics**: 
  <div align="center"> <img src="Plots/eof_trajectory_tracking1.jpg" width="400"/> <p align="center"> <em>End-Effector Trajectory Tracking in 3D Space</em> </p> </div>
  This provides the end-effector's position based on joint angles. The figure above illustrates the robot's reachable workspace.

- **Inverse Kinematics**:
  The inverse kinematics were derived geometrically, solving for joint angles based on the desired end-effector position. Inspired by the paper, equations for joint angles are solved analytically, ensuring precise configurations.

### **Control System**
The PD controller ensures smooth and accurate trajectory tracking:
- **Proportional-Derivative Control**:
  A PD controller was designed to track the desired trajectory with high precision. The control law minimizes the error between desired and actual positions while ensuring stability.

  ![Joint Angles vs Time](https://github.com/adharsh-prasad/Robotics-Portfolio/blob/main/PD-Control-Robotic-Arm/Plots/joint_angles.png){: width="30%"}

  The controller gains were tuned by experimentation, inspired by the method outlined in the paper. Unlike the paper, the implementation avoids PSO but achieves comparable results through iterative gain refinement.

- **Performance Testing**:
  Simulation results were validated using MATLAB. The plots above show the joint angles and velocities over time, highlighting the controller's smooth and accurate performance.

### **Dynamic Model**
The robotic arm's dynamics are represented as:
\[
M(q)\ddot{q} + V(q, \dot{q}) + G(q) = \tau
\]
Where:
- \(M(q)\): Inertia matrix.
- \(V(q, \dot{q})\): Coriolis and centripetal forces.
- \(G(q)\): Gravitational force.
- \(\tau\): Control torque.

This equation forms the basis of the PD control law:
\[
\tau = M(q)(\ddot{q}_d - K_p e - K_d \dot{e}) + V(q, \dot{q}) + G(q)
\]

---

## **Simulation and Results**

### **Trajectory Tracking**
<div align="center">
  <img src="Plots/end_effector_position.png" width="600"/>
  <p align="center">
    <em>End-Effector Position Tracking</em>
  </p>
</div>

The PD controller achieves excellent trajectory tracking, with minimal tracking errors as shown in the end-effector position tracking plots.

### **Singularity and Workspace Analysis**
Singularity analysis ensures safe motion planning within the robot's workspace. The workspace is visualized across multiple views:
- ![Reachable Workspace View 2](https://github.com/adharsh-prasad/Robotics-Portfolio/blob/main/PD-Control-Robotic-Arm/Plots/reachable_workspace_view2.jpg){: width="30%"}
- ![Reachable Workspace View 3](https://github.com/adharsh-prasad/Robotics-Portfolio/blob/main/PD-Control-Robotic-Arm/Plots/reachable_workspace_view3.jpg){: width="30%"}

---

## **Impact**

This project had a profound impact on my academic and professional growth:
- **Academic Recognition**: 
  - Earned top project recognition in a peer-reviewed class evaluation.
- **Professional Development**: 
  - Helped secure a position at the Space Robotics Lab at the University of Arizona.
- **Personal Growth**: 
  - Reinforced my passion for robotics and control systems through hands-on application and dynamic simulations.

---

## **Future Work**

1. **Optimization**:
   - Implementing Particle Swarm Optimization (PSO) for real-time controller parameter tuning.
2. **Expanded Applications**:
   - Extending the control system to multi-DOF robotic arms.
3. **Adaptive Control**:
   - Introducing adaptive controllers for better performance under uncertain conditions.

---

## **Installation and Usage**

### **Prerequisites**
- MATLAB with Simulink (R2022a or later).
- Robotics Toolbox for MATLAB.

### **Setup**
1. Clone the repository:
   ```bash
   git clone https://github.com/adharsh-prasad/trajectory-tracking-robotic-arm.git
2. Navigate to the project directory:
   ```bash
   cd trajectory-tracking-robotic-arm
3. Open the MATLAB scripts to run the simulations.

### **Reference**
1. Craig, J. J., "Introduction to Robotics Mechanics and Control", Pearson Education International, ISBN 0-13-123629-6, 2005.
2. Salah Mahdi Swadi et al., "Design and Simulation of Robotic Arm PD Controller Based on PSO", University of Thi-Qar Journal for Engineering Sciences, 2019.

### **Acknowledgments**
Special thanks to my professors, peers, and the Space Robotics Lab at the University of Arizona for their guidance and support in this project. This work represents a significant step forward in my journey as a robotics engineer.
