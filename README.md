ROS2 and Docker files for the course AIS2105 Mechatronics and Automation

# Installation
After installing docker (and WSL2 if you are using Windows), start up the docker engine.

Make sure that you have set up SSH on your computer:
```
ssh-add -l
```
If not, set up SSH, For instance using this:
(https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)[https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent]

Fork and clone this git repository and navigate to the folder with a terminal. (For instance in VSCode, open this folder as a working folder, and open a new terminal)

Run the following code and pray:
```
docker compose up
```
