# Lab Assignment 2: Workspaces and Transform Matrices
The purpose of this lab assignment is to learn how to set up a robot workspace and have the robot move around in them. This will specifically be that you first calculate different coordinate frames, and afterwards create a URDF which defines the workarea around the robot.

## Task 1: Transform Matrices
In the image below you there are four reference frames. {A} is the workspace frame, {B} is the gripper (or end-effector) frame, {C} is the camera frame and {D} is the workpiece frame. 

![Workarea](workarea.png)

### Task 1a
Using the dimensions  given in the figure, find the transform matrix $`\mathbf{T}_\text{ad}`$, $`\mathbf{T}_\text{ac}`$ and $`\mathbf{T}_\text{cd}`$

### Task 1b
Find the inverse $`\mathbf{T}_\text{ca}`$, and show that $`\mathbf{T}_\text{ca}\mathbf{T}_\text{ad}=\mathbf{T}_\text{cd}`$.

### Task 1c
Given the matrix
$$
\mathbf{T}_\text{bc} = \left[
    \begin{matrix}
        1 & 0 & 0 & 4 \\
        0 & 1 & 0 & 0 \\
        0 & 0 & 1 & 0 \\
        0 & 0 & 0 & 1
    \end{matrix}
\right]
$$
find the transform matrix $\mathbf{T}_\text{ab}$

## Task 2: Denavit-Hartenberg Table
In the image below, you see six reference frames that are aligned with the joints of a UR5 robot.

![UR5 with reference frames](ur5_frames.png)

Use the frames in the image the technical drawing on page 3 [here](https://www.synerbotwelding.com/wp-content/uploads/2020/09/area-de-trabajo-de-ur5.pdf) and make a Denavit-Hartenberg table for a UR5 robot.

## Task 3: Understanding URDF
Every robot in ROS2 is described using URDF (Universal Robot Description Format). It is XML based and describes each `link` and `joint`, where a link is a part of the robot (such as the shoulder, wrist, etc), while the joint is the joint between two links.

### Task 3a
Go through the tutorial given in [https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html] to create a simple table. You do not have to follow all the tutorials to achieve this.

The table should consist of two links: One the size of the robot table in the lab and one the size of the adapter plate the robot is mounted on

Create joints so that these two are places correctly relative to each other so that they are the same as the tables in the lab. Remember to create a joint between the `table` link and the `world` link

### Task 3b
Lastly, use the xacro commands to add a robot to the adapter plate (you can choose which model you want). Some commands that are useful are:
```
  <xacro:include filename="$(find ur_description)/urdf/ur_macro.xacro"/>
```
```
   <xacro:arg name="tf_prefix" default="" />
   <xacro:arg name="joint_limit_params" default="$(find ur_description)/config/ur5/joint_limits.yaml"/>
   <xacro:arg name="kinematics_params" default="$(find ur_description)/config/ur5/default_kinematics.yaml"/>
   <xacro:arg name="physical_params" default="$(find ur_description)/config/ur5/physical_parameters.yaml"/>
   <xacro:arg name="visual_params" default="$(find ur_description)/config/ur5/visual_parameters.yaml"/>
   <xacro:arg name="transmission_hw_interface" default=""/>
   <xacro:arg name="safety_limits" default="false"/>
   <xacro:arg name="safety_pos_margin" default="0.15"/>
   <xacro:arg name="safety_k_position" default="20"/>
```
```
   <xacro:ur_robot
     tf_prefix="$(arg tf_prefix)"
     parent="adapter_link"
     joint_limits_parameters_file="$(arg joint_limit_params)"
     kinematics_parameters_file="$(arg kinematics_params)"
     physical_parameters_file="$(arg physical_params)"
     visual_parameters_file="$(arg visual_params)"
     safety_limits="$(arg safety_limits)"
     safety_pos_margin="$(arg safety_pos_margin)"
     safety_k_position="$(arg safety_k_position)"
     force_abs_paths="$(arg force_abs_paths)"
     >
     <origin xyz="0 0 0" rpy="0 0 0" />          <!-- position robot in the world -->
   </xacro:ur_robot>
</robot>
```