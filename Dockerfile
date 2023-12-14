FROM osrf/ros:humble-desktop

# General Utilities
RUN apt update
RUN apt install -y x11-apps && \
    apt install -y mesa-utils
RUN apt install -y net-tools iputils-ping
RUN apt install -y wget

# ROS 2 installs
RUN apt install -y ros-humble-rmw-cyclonedds-cpp 
RUN apt install -y ros-humble-rviz2
RUN apt install -y ros-humble-ros2-control && \
    apt install -y ros-humble-ros2-controllers
RUN apt install -y ros-humble-moveit

# Set up DDS with host
ENV RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
ENV HOST_ADDR="1.2.3.4"

ENV CYCLONEDDS_URI="<CycloneDDS><Domain id='any'><General><ExternalNetworkAddress>${HOST_ADDR}</ExternalNetworkAddress><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>1</ParticipantIndex><Peers><Peer address='${HOST_ADDR}'/></Peers></Discovery><Tracing><Verbosity>config</Verbosity><Out>stderr</Out></Tracing></Domain></CycloneDDS>"


RUN echo ". /opt/ros/humble/setup.bash" >> ~/.bashrc

# ROS2 repos
COPY src /ros2_ws/src
RUN cd /ros2_ws/src

#RUN git clone git@github.com:lbr-stack/lbr_fri_ros2_stack.git
#RUN git clone -b ros2 git@github.com:fzi-forschungszentrum-informatik/cartesian_controllers.git
#RUN rosdep install --from-paths ./ --ignore-src -y
#RUN cd ..
#RUN colcon build --packages-skip cartesian_controller_simulation cartesian_controller_tests --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY entrypoint.sh /setup/entrypoint.sh

ENTRYPOINT ["/setup/entrypoint.sh"]