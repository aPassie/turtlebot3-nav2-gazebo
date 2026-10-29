# ROS 2 Jazzy (LTS) + Gazebo Harmonic + Nav2 + TurtleBot3
FROM osrf/ros:jazzy-desktop AS deps

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-nav2-minimal-tb3-sim \
        ros-jazzy-nav2-simple-commander \
        ros-jazzy-ros-gz \
        ros-jazzy-turtlebot3-msgs \
        ros-jazzy-turtlebot3-description \
        mesa-utils \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

FROM deps AS app
WORKDIR /ws
COPY tb3_nav2_demo /ws/src/tb3_nav2_demo
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

ENV TURTLEBOT3_MODEL=waffle \
    QT_QPA_PLATFORM=xcb

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
 && echo 'source /ws/install/setup.bash' >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
