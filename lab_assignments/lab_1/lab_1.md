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

If you run the command
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

## Task 2: Controlling a robot
Stop all the `talker`s and `listener`s and run the command

