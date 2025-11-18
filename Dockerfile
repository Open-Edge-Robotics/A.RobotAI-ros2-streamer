FROM docker.io/ros:humble

WORKDIR /ws/src

COPY src/ros_stream_server/ ros_stream_server/

RUN apt update && apt install -y net-tools neovim libgl1-mesa-glx python3-pip ros-humble-rmw-cyclonedds-cpp

RUN pip3 install ultralytics pyzmq cv_bridge 

WORKDIR /ws

RUN rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install
# RUN rm -fr src

COPY ros_entrypoint.sh /ros_entrypoint.sh
COPY former_best.pt ./former_best.pt

EXPOSE 15555
EXPOSE 15556
EXPOSE 15557

ENTRYPOINT [ "/ros_entrypoint.sh" ]

CMD [ "/bin/bash", "-c", ". /opt/ros/${ROS_DISTRO}/setup.sh; ros2 launch ros_stream_server stream_server.launch.py" ]