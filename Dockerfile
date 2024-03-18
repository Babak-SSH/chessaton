FROM osrf/ros:humble-desktop-full
MAINTAINER Kevin DeMarco
ENV DEBIAN_FRONTEND noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && apt-get install -y \
    python3-pip \
    ros-humble-gazebo-ros \
    && rm -rf /var/lib/apt/lists/*

# Create the "ros" user, with the host user' IDs
ARG USER_ID=1000
ARG GROUP_ID=1000

ENV USERNAME ros
RUN adduser --disabled-password --gecos '' $USERNAME \
    && usermod  --uid ${USER_ID} $USERNAME \
    && groupmod --gid ${GROUP_ID} $USERNAME \
    && usermod --shell /bin/bash $USERNAME \
    && adduser $USERNAME sudo \
    && adduser $USERNAME dialout \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER $USERNAME

# Run rosdep update, add ROS, Gazebo, and colcon setup to ros user's .bashrc
RUN sudo apt-get update \
    && rosdep update \
    && echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /usr/share/colcon_cd/function/colcon_cd.sh' >> /home/$USERNAME/.bashrc

# Create the workspace
RUN mkdir -p /home/$USERNAME/workspace
WORKDIR /home/$USERNAME/workspace

# Copy code into workspace, run rosdep install for workspace, build, and source setup in ros user's
COPY --chown=ros ./src src
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && sudo rosdep install --from-paths . --ignore-src -r -y --rosdistro=${ROS_DISTRO} \
    && colcon build --symlink-install --parallel-workers=2\
    && echo 'source ~/workspace/install/local_setup.bash' >> /home/$USERNAME/.bashrc
