FROM docker.io/ros:humble

WORKDIR /ws/src

COPY . ros_stream_server/

RUN apt update && apt install -y net-tools neovim libgl1-mesa-glx python3-pip ros-humble-rmw-cyclonedds-cpp

RUN pip3 install opencv-python pyzmq cv_bridge

WORKDIR /ws

RUN rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build
RUN rm -fr src

COPY ros_entrypoint.sh /ros_entrypoint.sh

EXPOSE 15555
EXPOSE 15556

ENTRYPOINT [ "/ros_entrypoint.sh" ]

CMD [ "/bin/bash", "-c", ". /opt/ros/${ROS_DISTRO}/setup.sh; ros2 launch ros_stream_server stream_server.launch.py" ]