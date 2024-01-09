ROS2 and Docker files for the course AIS2105 Mechatronics and Automation

# Installation
After installing docker (and WSL2 if you are using Windows), start up the docker engine.

Make sure that you have set up SSH on your computer:
```
ssh-add -l
```
If not, set up SSH, For instance using this:
https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent

Fork and clone this git repository and navigate to the folder with a terminal. (For instance in VSCode, open this folder as a working folder, and open a new terminal)

Run the following code and pray:
```
docker compose up
```

# Running Docker
Every time you want to use this project, run
```
docker compose up
```

If you're using VSCode, click the small green icon in the lower left corner. Click then "Attack to Running Container" and select the running container. A new window pops up, which will be your main development platform.

The ´src´ folder on your computer is linked to the ´/workspaces/ros2_ws/src´ folder inside your docker container, so any files changed or added in either place will be synced.