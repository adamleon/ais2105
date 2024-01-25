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

## On Windows
>> Before running the code, check that the `entrypoint.sh` file is has a line ending of LF (Line Feed) not CRLF (Carrige Return, Line Feed). To do this (in VSCode at least) open the file and look in the lower right corner. It says somthing about line and column position of the cursor, number of spaces, UTF-8, etc. There it says LF or CRLF. Click that and change to LF. Save the file

Run the following code and pray:
```
docker compose up windows-wsl
```
If you have Nvidia instead run
```
docker compose up windows-wsl-nvidia
```

## On Mac 
Run the following code:
```
brew install xquartz
```
and restart your PC
After that run
```
xhost +
```
Open XQuartz (which should be running), go into Settings/Preferences->Security->allow connections from network clients

Run the following code and pray:
```
docker compose up mac
```
There is a problem that OpenGL does not work because docker cannot find a graphics driver. If you find a solution, contact me.

# On Linux
Most modern Linux distribution have some form of authentication when connecting to the X11 server. Therefore docker will need some way to authenticate their permission to open GUI applications.
Run the following on your host system first to allow for GUI applications:
```
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f .docker.xauth nmerge -
```

Ignore below output, it is expected:
```
xauth:  file .docker.xauth does not exist
```

Run the following and pray:
```
docker compose up linux
```

You might need root permissions to run a container, this can be avoided by adding your user to the docker group:
```
sudo usermod -a -G docker $USER
```

# Running Docker
Every time you want to use this project, run
```
docker compose up windows-wsl/windows-wsl-nvidia/mac/linux
```

If you're using VSCode, click the small green icon in the lower left corner. Click then "Attack to Running Container" and select the running container. A new window pops up, which will be your main development platform.

The `src` folder on your computer is linked to the `/workspaces/ros2_ws/src` folder inside your docker container, so any files changed or added in either place will be synced.
