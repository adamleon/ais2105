# Lab Assignment 1: Publish and Subscribe
The purpose of this lab assignment is to learn how to use ROS2, specifically how to send and receive data, and how to create programs to manage it.

## Task 1: Getting Started
After starting up the docker container and attaching to the container with your IDE (see Running Docker in the README file), you should be able to have a terminal and a working space which runs the ROS2 virtual machine.

To see if ROS2 is running correctly, run
```
ros2 run demo_nodes_py talker
```
You will see that the `talker` node starts publishing "Hello world" messages

Start up another terminal (in your IDE) and run
```
ros2 run demo_nodes_py listener
```
You will see that the `listener` node starts reading the messages published from the `talker` node.

If you run the command (in yet another terminal)
```
ros2 topic list
```
You get a list of all the topics available on the network. You will see the `\chatter` topic listed, which is the topic which `talker` is **publishing** messages from, while `listener` **subscribes** to the `/chatter` topic.

Run
```
ros2 topic info /chatter
```
You see more information about the `/chatter` topic. It sends messages of type `std_msgs/msg/String`, which is only a string. You can also see how many are subscribing and publishing to this topic, which should be 1 for both if you have `talker` and `listener` running. You can run more `talker`s and `listener`s simultaneously and see what happens.

You can run the command
```
rqt_graph
```
which shows who are connected to who with topics.

## Task 2: Making a publisher and subscriber
Follow the tutorial here to make a publisher and subscriber 

[Python Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

[C++ Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)

If you're unsure, Python is probably easiest.

If you want an introductory tutorial to ROS2, you can also follow these:
[Understanding ROS2](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html#)
[Creating a workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
[Creating a package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

### Monitoring a numerical message
The publisher/subscriber tutorials describe how to make a `talker` and `listener`. We will now try to make this more useful.

A problem in ROS2 (which we will solve now) is to draw graphs of a robot's joints. The problem being that when `rqt` listens to messages, it only accepts to draw Float64 values, while the robot's joints are published as an array of Float64.

Your task is to create a new node, which subscribes to the robot's joints and publishes each of the 6 values into six different topics, so that the `rqt` program can subscribe to each of the six and plot them in a graph. But, let's take it one step at a time.

### Creating six publishers
Stop all the `talker`s and `listener`s (``ctrl + C` in every terminal) and continue reading.

Create a new node and follow the same steps as in the tutorial where you made a publisher, except give it a new name (your choice, but "joint_value_splitter" is an example).

1. You now must change the publisher so that it doesn't use `std_msgs/msg/String`, but `std_msgs/msg/Float64`, and change the publisher to publish a number rather than a string (use a random number, or sine function or something). Also change the name of the publisher topic to "shoulder_pan_joint".
2. Build the node and check that it runs correctly
3. Run `rqt` in a new terminal, and under `plugins` you find a visualization plugin called Plot.
4. From there find the topic "shoulder_pan_joint/data" and click the "+" symbol to put it in the graph. You should now see the graph update.
5. Now, you can extend the number of publishers. Make five new publishers, each which publishes some number function (your choice). Name the topics "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint" and "wrist_3_joint".
6. Inside `rqt` use the "+" to add all five publishers.

Now that you have a system which publishes six values, we will continue to create a subscriber.

## Task 3: Monitoring a robot
### Simulation setup
Start a new terminal and run the command
```
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py
```

This will launch two windows (and a lot of other stuff): RViz and Gazebo.

Gazebo is a physics simulator, which in our case simulates a UR robot, including kinematics and dynamics. We can explore this in a later lab assignment.

RViz is a visualization tool, which in our case visualizes the UR robot. This is generally our interface into controlling the robot. RViz takes the current measurements from the robot and displays what the robot looks like. Currently it shows the same as the simulator, but when connecting to a real robot, it will show the robots position.

Run now the command in a new terminal
```
ros2 launch ur_robot_driver test_joint_trajectory_controller.launch.py
```
The `ur_robot_driver` is the control interface which sends commands to the robot, and the `test_joint_trajectory_controller` will sets up a simple program that moves the robot between 4 different poses. You can see now that the simulation iterates through the four poses, and that RViz mirrors the movements.

If you think the windows are slow and in the way, you can use `ctrl+C` to terminate the programs. If you want to start the simulation and movement again you run the commands
```
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py
```
and
```
ros2 launch ur_robot_driver test_joint_trajectory_controller.launch.py
```

### Creating a subscriber
Next we will add a subscriber to your node. Look at the subscriber tutorial for inspiration and make the following:

1. Initialize a subscriber that takes in a trajectory_msgs/msg/JointTrajectoryMsg. Remember to use import (Python) or include (C++), and to update the `packages.xml` (and CMakeLists.txt in C++).
2. Change the `listener_callback` so that it takes in a JointTrajectoryMsg and reads `points.position[i]` where `i` refers to a specific joint (0 is "shoulder_pan_joint", 1 is "shoulder_lift_joint", etc.). Read each of the values and publish them on the correct publishing topic
3. Build the node and run the simulator and plotter to see what happens.