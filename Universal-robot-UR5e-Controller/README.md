# **UR5e Robotic Arm Advanced Trajectory Tracking with MPC**


## **Motivation**

During my internship at SpaceTREx, I worked hands-on with the UR5e Universal Robots industrial arm for a block assembly task using miniature sandbags. Operating the arm via UR’s HMI teach pendant—with its intuitive waypoint programming and real-time force/torque monitoring—gave me firsthand insight into industrial robotics. Observing robust collision avoidance and singularity handling in real-time operations inspired me to explore advanced control strategies for complex robotic systems.

This experience motivated me to adapt my prior [PD-controlled trajectory tracking](https://github.com/adharsh-prasad/Robotics-Portfolio/tree/main/PSO-PD-Control) work to the UR5e platform using Model Predictive Control (MPC). While classical controllers excel in many scenarios, MPC’s predictive optimization and constraint-handling capabilities make it ideal for high-precision industrial applications. Implementing MPC on this 6-DOF arm deepened my understanding of advanced control theory while bridging academic concepts with real-world robotic challenges.

<div align="center"> <img src="Plots/Anthropomorphic Arm.png" width="200"/> <p align="center"> <em>Anthropomorphic arm configuration used for our analysis, with reasonable complex structure while omitting the end effector.</em> </p> </div>

---

## **Features**

<div align="center"> <img src="Plots/Skeleton of the robotic arm.png" width="300"/> <p align="center"> <em> Detailed configuration of the arm with Co-ordinate axis for each joint of the arm</em> </p> </div>

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
  - Manual fine-tuning was performed for the PD controller gains.
  - Further optimization using Particle Swarm Optimization (PSO) can be explored in the [PSO-PD Control Project](https://github.com/adharsh-prasad/Robotics-Portfolio/tree/main/PSO-PD-Control).

---

## **Technical Approach**

### **Kinematic Model**
Forward and inverse kinematics were developed to enable precise motion planning:
- **Forward Kinematics**: 
  <div align="center">
      <table>
        <tr>
          <th style="background-color: green; color: white;">Link</th>
          <th style="background-color: green; color: white;">$r_i$ (m)</th>
          <th style="background-color: green; color: white;">$\alpha_i$ (deg)</th>
          <th style="background-color: green; color: white;">$d_i$ (m)</th>
          <th style="background-color: green; color: white;">$\theta_i$ (deg)</th>
        </tr>
        <tr>
          <td style="background-color: #fce8b2;">$1$</td>
          <td style="background-color: #e6f4ea;">$0$</td>
          <td style="background-color: #fbe4e6;">$\pi/2$</td>
          <td style="background-color: #fbe4e6;">$a_1$</td>
          <td style="background-color: #fbe4e6;">$\theta_1$</td>
        </tr>
        <tr>
          <td style="background-color: #fce8b2;">$2$</td>
          <td style="background-color: #e6f4ea;">$a_2$</td>
          <td style="background-color: #fbe4e6;">$0$</td>
          <td style="background-color: #fbe4e6;">$0$</td>
          <td style="background-color: #fbe4e6;">$\theta_2$</td>
        </tr>
        <tr>
          <td style="background-color: #fce8b2;">$2$</td>
          <td style="background-color: #e6f4ea;">$a_2$</td>
          <td style="background-color: #fbe4e6;">$0$</td>
          <td style="background-color: #fbe4e6;">$0$</td>
          <td style="background-color: #fbe4e6;">$\theta_3$</td>
        </tr>
      </table>
    <p align="center"> <em> DH table for our chosen robotic arm configuration</em> </p>
    </div>
    
The transformation matrix from joint(i-1) to joint(i) is defined as:

$$
A_{i-1}^i =
\begin{bmatrix}
cos(\theta_i) & -sin(\theta_i)cos(\alpha_{i-1}) & sin(\theta_i)sin(\alpha_{i-1}) & a_{i-1}cos(\theta_i)\\
sin(\theta_i) & cos(\theta_i)cos(\alpha_{i-1}) & -cos(\theta_i)sin(\alpha_{i-1}) & a_{i-1}sin(\theta_i)\\
0 & sin(\alpha_{i-1}) & cos(\alpha_{i-1}) & d_i\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$ T_{3}^0 = A_{0}^1 * A_{1}^2 * A_{2}^3$$ 

Upon substituting the corresponding values of the a_i 's, alpha_i's, d_i's, and theta_i's. We get position of the end effector as

$$
P_{3}^0 =
\begin{bmatrix}
cos(\theta_1)(a_2cos(\theta_2) + a_3cos(\theta_2+\theta_3))\\
sin(\theta_1)(a_2cos(\theta_2) + a_3cos(\theta_2+\theta_3))\\
a_1 + a_2sin(\theta_2) + a_3sin(\theta_2+\theta_3))
\end{bmatrix}
$$

- **Inverse Kinematics**:
  The inverse kinematics were derived geometrically, solving for joint angles based on the desired end-effector position. Finding the inverse kinematics is quite challenging and often does not have a unique solution. For the chosen configuration, solving the algebraic equations can be complex and is not recommended. The geometric method is more intuitive and easier to solve. The diagrams below help to find the joint angles for a specific end-effector point in the 3D plane.

  <div align="center"> <img src="Plots/Joint_1_Inverse.png" width="500"/> <p align="center"> <em> Diagram showing how to find the joint angle joint1 </em> </p> </div>

  <div align="center"> <img src="Plots/Joint_23_Inverse.png" width="500"/> <p align="center"> <em> Diagram showing how to find the joint angle joint2 and joint3 </em> </p> </div>

### **Control System**
The PD controller ensures smooth and accurate trajectory tracking:
- **Proportional-Derivative Control**:
  A PD controller was designed to track the desired trajectory with high precision. The control law minimizes the error between desired and actual positions while ensuring stability.
  The controller gains were manually tuned by varying kp and kd for all three joints. Although the resulting errors were relatively high, further trial and error could have reduced them. However, since this project focused on the dynamics and control of the robot, it served its purpose. In another project using the same robotic arm, I implemented PSO optimization to achieve an RMSE error as low as 0.004. If you're interested in exploring that project, please visit my [PSO-PD Control Project](https://github.com/adharsh-prasad/Robotics-Portfolio/tree/main/PSO-PD-Control).


- **Performance Testing**:
  Simulation results were validated using MATLAB. The plots under the "Simulation and Results" section below shows the joint angles and velocities over time, highlighting the controller's performance.

### **Dynamic Model**
The robotic arm's dynamics are represented as:

$$
M(q)\ddot{q} + V(q, \dot{q}) + G(q) = \tau
$$

Where:

$$ M(q): \quad \text{Inertia matrix,} $$
$$ V(q, \dot{q})\: \quad \text{Coriolis and centripetal forces,} $$
$$ G(q)\: \quad \text{Gravitational force,} $$
$$ \tau\: \quad \text{Control torque.} $$

This equation forms the basis of the PD control law:

$$
\tau = M(q)(\ddot{q}_d - K_p e - K_d \dot{e}) + V(q, \dot{q}) + G(q)
$$

---

## **Simulation and Results**

### **Trajectory Tracking**

<div align="center">
  <img src="Plots/end_effector_position.png" width="450" alt="End Effector Position">
  <img src="Plots/eof_trajectory_tracking1.jpg" width="450" alt="Trajectory Tracking 1">
</div>
<div align="center">
  <img src="Plots/eof_trajectory_tracking2.jpg" width= "450" alt="Trajectory Tracking 2">
  <img src="Plots/eof_trajectory_tracking3.jpg" width="450" alt="Trajectory Tracking 3">
</div>

The PD controller achieves trajectory tracking to some extent, with significant tracking errors as shown in the end-effector position tracking plots. The error is predominantly due to the fact there are not fine-tuned enough to achieve minimum error. Again the minimum error was achieved with the PSO parameter tuning, please visit my [PSO-PD Control Project](https://github.com/adharsh-prasad/Robotics-Portfolio/tree/main/PSO-PD-Control).

### **Joint Angle Analysis**

<div align="center">
  <img src="Plots/joint_angles.png" width="450" alt="Joint angle tracking">
  <img src="Plots/joint_angular_velocities.png" width="450" alt="Joint angle angular velocity tracking">
</div>

The initial tracking errors significantly impact the overall performance of the trajectory tracking. Although the velocity plots indicate that the error eventually approaches zero, the large initial error propagates through the system, adversely affecting joint angle tracking performance. The plot below of the joint angle errors provides additional insights into the previously discussed inference.

<div align="center">
  <img src="Plots/joint_angle_tracking_error.png" width="450" alt="Joint angle tracking">
  <img src="Plots/joint_angular_velocity_errors.png" width="450" alt="Joint angle angular velocity tracking">
</div>

### **Control Torque Plot**

<div align="center">
  <img src="Plots/control_torques.png" width="700" alt="Control Torque from each joints">
</div>

### **Workspace Analysis**
The trajectory is completely user-defined, and you are welcome to change it in the [simulation file](https://github.com/adharsh-prasad/Robotics-Portfolio/blob/main/PD-Control-Robotic-Arm/Code/Simulation.m). However, be careful to choose trajectories such that all points are within the workspace. The workspace can be visualized in the image below (files for these plots are available in the [plots folder](https://github.com/adharsh-prasad/Robotics-Portfolio/tree/main/PD-Control-Robotic-Arm/Plots)) and can also be visualized with the provided code.

<div align="center">
  <img src="Plots/reachable_workspace_view1.jpg" width= "450" alt="Reachable Workspace view 1">
  <img src="Plots/reachable_workspace_view2.jpg" width="450" alt="Reachable Workspace view 2">
</div>

<div align="center">
  <img src="Plots/reachable_workspace_view3.jpg" width= "450" alt="Reachable Workspace view 3">
</div>

### **Simulation Video**

The following video demonstrates the complete simulation of the 3-DOF robotic arm performing trajectory tracking. It showcases the arm's smooth motion and the effectiveness of the implemented PD controller in achieving accurate trajectory tracking.

<div align="center"> <img src="Plots/Robotic-Arm-Simulation.gif"> <p align="center"> <em>This simulation highlights the integration of kinematics, dynamics, and control theory, providing a visual representation of the robotic arm's performance.</em> </p> </div>


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
   - Implementing Particle Swarm Optimization (PSO) for real-time controller parameter tuning([visit](https://github.com/adharsh-prasad/Robotics-Portfolio/tree/main/PSO-PD-Control)).
2. **Expanded Applications**:
   - Extending the control system to multi-DOF robotic arms.
3. **Adaptive Control**:
   - Introducing adaptive controllers for better performance under uncertain conditions.

---

## **Try It Yourself**

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
   The code is designed to be user-friendly, allowing you to run any trajectory within the workspace with minimal setup. Change the x_func, y_func and z_func as per the users need. Simply execute the following lines in MATLAB after making these changes:
   ```bash
       [robot, arm_length] = Robotic_arm_model();
       x_func = @(t) (35 + 25*cos(2*pi*t/10))/100;
       y_func = @(t) (35 + 25*sin(2*pi*t/10))/100; 
       z_func = @(t) 90/100*t.^0;
       [T, joint_angles] = Joint_trajectory(x_func, y_func, z_func, robot, arm_length);

  Note: Ensure that your chosen trajectory remains within the defined workspace. Details of the workspace are explained above.

### **Customization**

If you wish to modify the robot's dimensions or parameters, you can do so by editing the [Robotic_arm_model.m](https://github.com/adharsh-prasad/Robotics-Portfolio/blob/main/PD-Control-Robotic-Arm/Code/Robotic_arm_model.m) file. Adjust any desired parameters to fit your specific requirements. This section provides a straightforward way for users to engage with and explore the simulation, emphasizing ease of use and customization potential.

### **Reference**
1. Craig, J. J., "Introduction to Robotics Mechanics and Control", Pearson Education International, ISBN 0-13-123629-6, 2005.
2. Salah Mahdi Swadi et al., "Design and Simulation of Robotic Arm PD Controller Based on PSO", University of Thi-Qar Journal for Engineering Sciences, 2019.

### **Acknowledgments**
Special thanks to my professors, peers for their guidance and support in this project. Special mention to Space Robotics Lab at the University of Arizona for recognizing my project and offering a research internship. This work represents a significant step forward in my journey as a robotics engineer.
