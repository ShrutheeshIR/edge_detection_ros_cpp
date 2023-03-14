FROM ubuntu:20.04
# include ros noetic
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
   apt-get install -y --no-install-recommends \
         git \
        cmake \
        build-essential \
        curl \
        wget \
        gnupg2 \
        lsb-release \
        ca-certificates \
      net-tools \
  && rm -rf /var/lib/apt/lists/*
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && \
   apt-get install -y --no-install-recommends \
        ros-noetic-desktop-full \
        ros-noetic-image-transport \
        ros-noetic-vision-msgs \
        ros-noetic-message-filters \
         python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool \
   && rm -rf /var/lib/apt/lists/*
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc
# RUN echo 'export ROS_MASTER_URI=https://localhost:11311' >> /root/.bashrc
# RUN echo 'export ROS_HOSTNAME=localhost' >> /root/.bashrc

RUN apt-get install -y libopencv-dev
RUN apt-get update && apt-get install -y python3-catkin-tools