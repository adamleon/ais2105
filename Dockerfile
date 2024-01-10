FROM osrf/ros:humble-desktop
# SHELL ["/bin/bash", "-c"]

# General Utilities
RUN apt update
RUN apt install -y x11-apps && \
    apt install -y mesa-utils
RUN apt install -y net-tools iputils-ping
RUN apt install -y wget
RUN apt install -y openssh-client git

# download public key for github.com
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts
#RUN --mount=type=ssh bundle install

# OpenGL bug fix
RUN apt install -y software-properties-common
RUN add-apt-repository ppa:kisak/kisak-mesa
RUN apt update
RUN apt upgrade -y

# ROS 2 installs
RUN apt install -y ros-humble-rviz2
RUN apt install -y ros-humble-ros2-control && \
    apt install -y ros-humble-ros2-controllers && \
    apt install -y ros-humble-rqt-joint-trajectory-controller
RUN apt install -y ros-humble-moveit
RUN apt install -y ros-humble-ur

# Set up DDS with host
#ENV RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
#ENV HOST_ADDR="1.2.3.4"

#ENV CYCLONEDDS_URI="<CycloneDDS><Domain id='any'><General><ExternalNetworkAddress>${HOST_ADDR}</ExternalNetworkAddress><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>1</ParticipantIndex><Peers><Peer address='${HOST_ADDR}'/></Peers></Discovery><Tracing><Verbosity>config</Verbosity><Out>stderr</Out></Tracing></Domain></CycloneDDS>"

RUN rosdep update
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set up 3rd party workspace
ENV THIRD_PARTY_WS  /workspaces/3rd_party_ws
RUN mkdir -p $THIRD_PARTY_WS/src
WORKDIR $THIRD_PARTY_WS
RUN --mount=type=ssh git clone -b ros2 git@github.com:fzi-forschungszentrum-informatik/cartesian_controllers.git $THIRD_PARTY_WS/src/cartesian_controllers
# RUN --mount=type=ssh git clone git@github.com:UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git $THIRD_PARTY_WS/src/Universal_Robots_ROS2_Gazebo_Simulation
# RUN vcs import src --input src/Universal_Robots_ROS2_Gazebo_Simulation/Universal_Robots_ROS2_Gazebo_Simulation.humble.repos
RUN rosdep install --from-paths src --ignore-src --rosdistro humble -y 
RUN . /opt/ros/humble/setup.sh \
    && colcon build --packages-skip cartesian_controller_simulation cartesian_controller_tests --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN echo "source $THIRD_PARTY_WS/install/setup.bash" >> ~/.bashrc

# Set up main workspace
ENV MAIN_WS  /workspaces/ros2_ws
RUN mkdir -p $MAIN_WS/src
COPY src $MAIN_WS/src
WORKDIR $MAIN_WS

# WSL
ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib

COPY entrypoint.sh /setup/entrypoint.sh

ENTRYPOINT ["/setup/entrypoint.sh"]

SHELL ["/bin/bash", "-c"]