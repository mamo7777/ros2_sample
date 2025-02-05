FROM osrf/ros:jazzy-desktop


RUN apt-get update && apt-get install -y \
    tmux \
    vim \
    x11-apps \
    build-essential \
    libxcb-xinerama0 

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /root/ros2_sample/meitec_ros2_sample/install/setup.bash" >> /root/.bashrc

WORKDIR /root/ros2_sample

ENTRYPOINT ["/root/ros2_sample/scripts/entrypoint.sh"]
