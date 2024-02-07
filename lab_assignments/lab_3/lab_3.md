# Lab Assigment 3: Forward and Inverse Kinematis
The purpose of this lab assignment is to learn how to move a robot around, either by controlling the joint values or give it coordinates to move to.

## Task 1: Analytical Inverse Kinematics
See the figure below

![scara.jpg](scara.jpg)

Let $L_1=0.75\text{m}$, $L_2=0.50\text{m}$ and $L_3=0.20\text{m}$. Where $d_1$, $\theta_2$, $\theta_3$ and $\theta_4$ are joints that can move the robot.

### Task 1a
How many DoF does this robot have?

### Task 1b
What singularities does it have? And which ones are physically possible to reach and which are not?

### Task 1c
Assume that $d_1$ and $\theta_3$ are fixed, such that the gripper is touching the table, and that therefore $z=0$. We only focus on moving the robot along the $xy$-plane. Write an equation 

```math
\left[\begin{matrix}\theta_2\\\theta_3\end{matrix}\right] = f(x,y)
```

so that you can determine the correct $\theta_2$ and $\theta_3$ given an $(x,y)$ coordinate.

### Task 1d
Expand the equation $f(x,y)$ to 

```math
\left[\begin{matrix}d_1\\\theta_2\\\theta_3\\\theta_4\end{matrix}\right] = f(x,y,z,\phi)
```

such $d_1$ moves the arm up or down from 0 to 1 meter. $\theta_2$, $\theta_3$ and $\theta_4$ can rotate $360^\circ$.

## Task 2: Path Planning
Given the robot in task 1. Draw or somehow describe a path that can be reached through a straight line in joint space, but that cannot be reached through a linear motion.

A linear movement can be described with the equation
```math
\mathbf{X}(s)=\mathbf{X}_\text{start}+s(\mathbf{X}_\text{end}-\mathbf{X}_\text{start})
```
Where $s$ is a number between 0 and 1 inclusive, and $\mathbf{X}$ is
```math
\mathbf{X}=\left[\begin{matrix}x\\y\\z\end{matrix}\right]
```
Write a function $\mathbf{X}(s)$ which draws an arc from $\mathbf{X}_\text{start}$ to $\mathbf{X}_\text{end}$. The arc should lie on a circle with the center point $\mathbf{c}$, which is a vector with an $x$, $y$ and $z$ coordinate.

Hint: You can define $\mathbf{r}$ so that $\mathbf{X}_\text{start}=\mathbf{r}+\mathbf{c}$, meaning a vector from the center of the circle to the start point. The $\mathbf{r}$ vector can be rotated with an angle which will draw the arc and end at $\mathbf{X}_\text{end}=\mathbf{r}_\text{rot}+\mathbf{c}$. Finding the rotation axis and angle is the challenge.

## Task 3: Commanding a Robot
Find a robot which has a Microsoft Surface installed next to it. Turn on the robot and start up the Surface laptop. The password is "iir_robot".

### Task a: Starting ROS2 Control
Run the command in a new terminal
```
ros2 launch ur_robot_driver ur3e.launch.py robot_ip:=192.168.1.102
```
but change the "ur3e" to whichever robot model you are using.

Open a new terminal (or tab) and use the command
```
top
```
to see all running programs and find the "ur-ros2-control". Use "ctrl+C" to exit, and look at the PID number for "ur-ros2-control". Then run the code
```
chrt -p 99 <pid>
```
which will set the priorty of the program to 99.

Verify that the program is running in realtime by running
```
top
```
again and see that the Priority is set to "rt" for "ur_ros2_control".

On the robot teachpad, start up the robot and run the program "ROS2 Control.urscript".

Verify in the first terminal that the PC has established a connection with the robot.

### Task b: Moving the robot
In a new terminal run the command
```
ros2 control list_controllers
```
and verify that the "scaled_trajectory_joint_controller" is active. If not, run the command
```
ros2 control set_controller_state scaled_trajectory_controller active
```
Remember that you can autocomplete by pressing the tab key.

Move the robot to the joint angles [0,-90,0,-90,0,0] through the teachpad, go back to the program and press play.

Run the command
```
ros2 launch ur_robot_driver test_scaled_trajectory_joint_controller
```
and verify that the robot is moving.

### Task c: MoveIt
Exit the `test_scaled_trajectory_joint_controller` node and run the command
```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true 
```
and set the `ur_type` to whichever robot you are using.

In RViz use the marker to move the orange robot around. On the "MotionPlanning" panel on the left, go to the "Planning" tab and press the "Plan" button and make MoveIt plan a path for the robot. To loop the animation, look for "MotionPlanning" in the list in the "Display" panel, expand it and expand the "Planned Path" and check "Loop Animation".

If you think the planned path seems okay, press the "Execute" button and see the robot move.

If you wish to "restart" the marker, you can select "same as start" in the dropdown menu under Goal State.