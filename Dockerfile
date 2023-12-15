FROM osrf/ros:humble-desktop

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
RUN apt install -y ros-humble-rmw-cyclonedds-cpp 
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


RUN echo ". /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set up workspaces
RUN mkdir -p /workspaces/3rd_party_ws/src
RUN cd /workspaces/3rd_party_ws/src
RUN git clone -b ros2 git@github.com:fzi-forschungszentrum-informatik/cartesian_controllers.git
RUN rosdep install --from-paths ./ --ignore-src -y
RUN cd ..
RUN colcon build --packages-skip cartesian_controller_simulation cartesian_controller_tests --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY src /workspaces/ros2_ws/src
RUN cd /workspaces/ros2_ws/src

# WSL
ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib

COPY entrypoint.sh /setup/entrypoint.sh

ENTRYPOINT ["/setup/entrypoint.sh"]