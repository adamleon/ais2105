FROM osrf/ros:humble-desktop AS robot_dev_base

# General Utilities
RUN apt update
RUN apt install -y x11-apps && \
    apt install -y mesa-utils
RUN apt install -y net-tools iputils-ping
RUN apt install -y wget
RUN apt install -y openssh-client git

# download public key for github.com
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

# ROS2 Packages
RUN apt install -y ros-humble-rviz2
RUN apt install -y ros-humble-ros2-control && \
    apt install -y ros-humble-ros2-controllers && \
    apt install -y ros-humble-rqt-joint-trajectory-controller
RUN apt install -y ros-humble-moveit
RUN apt install -y ros-humble-ur
RUN apt install -y gazebo

# ROS2 Configurations
RUN rosdep update
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set up 3rd party workspace
ENV THIRD_PARTY_WS  /workspaces/3rd_party_ws
RUN mkdir -p $THIRD_PARTY_WS/src
WORKDIR $THIRD_PARTY_WS
# RUN --mount=type=ssh git clone -b ros2 git@github.com:fzi-forschungszentrum-informatik/cartesian_controllers.git $THIRD_PARTY_WS/src/cartesian_controllers
RUN --mount=type=ssh git clone -b humble git@github.com:UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git $THIRD_PARTY_WS/src/Universal_Robots_ROS2_Gazebo_Simulation
RUN --mount=type=ssh git clone -b humble git@github.com:ros-controls/gazebo_ros2_control.git $THIRD_PARTY_WS/src/gazebo_ros2_control

RUN rosdep install --from-paths src --ignore-src --rosdistro humble -y 
RUN . /opt/ros/humble/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN echo "source $THIRD_PARTY_WS/install/setup.bash" >> ~/.bashrc

# Set up main workspace
ENV MAIN_WS  /workspaces/ros2_ws
RUN mkdir -p $MAIN_WS/src
WORKDIR $MAIN_WS

COPY entrypoint.sh /setup/entrypoint.sh
ENTRYPOINT ["/setup/entrypoint.sh"]

SHELL ["/bin/bash", "-c"]

## Docker for WSL2 and NVidia drivers
FROM robot_dev_base AS robot_dev_wsl_nvidia

# Install CUDA
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/cuda-wsl-ubuntu.pin
RUN mv cuda-wsl-ubuntu.pin /etc/apt/preferences.d/cuda-repository-pin-600
RUN wget https://developer.download.nvidia.com/compute/cuda/12.3.2/local_installers/cuda-repo-wsl-ubuntu-12-3-local_12.3.2-1_amd64.deb
RUN dpkg -i cuda-repo-wsl-ubuntu-12-3-local_12.3.2-1_amd64.deb
RUN cp /var/cuda-repo-wsl-ubuntu-12-3-local/cuda-*-keyring.gpg /usr/share/keyrings/
RUN apt update
RUN apt -y install cuda-toolkit-12-3

RUN export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

# WSL
ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib

## Docker for WSL2
FROM robot_dev_base AS robot_dev_wsl

# WSL
ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib

## Docker for MacOS
FROM robot_dev_base AS robot_dev_mac

## Docker for Linux
FROM robot_dev_base AS robot_dev_linux