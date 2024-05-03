FROM osrf/ros:humble-desktop-full
MAINTAINER Babak-SSH
ENV DEBIAN_FRONTEND noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && apt-get install -y \
    python3-pip \
    ros-humble-gazebo-ros \
    && rm -rf /var/lib/apt/lists/*

# Create the "ros" user, with the host user' IDs
# ARG USER_ID=1000
# ARG GROUP_ID=1000

ENV USERNAME ros
ENV USER_ID 1000
ENV GROUP_ID 1000

RUN adduser --disabled-password --gecos '' $USERNAME \
    && usermod  --uid $USER_ID $USERNAME \
    && groupmod --gid $GROUP_ID $USERNAME \
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

# Install basic tools
RUN sudo apt install -y \
    build-essential \
    cmake \
    git \
    vim \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    wget

# Create the workspace
RUN mkdir -p /home/$USERNAME/ws_chessaton/src

# to built moveit and all its dependencies from source uncomment these lines (installing moveit from source needs atleast 16gb memory)
RUN git clone https://github.com/ros-planning/moveit_task_constructor.git -b $ROS_DISTRO /home/$USERNAME/ws_chessaton/src
    # && git clone https://github.com/ros-planning/moveit2.git -b $ROS_DISTRO \
    # && git clone https://github.com/ros-planning/moveit_msgs.git -b $ROS_DISTRO \
    # && git clone https://github.com/wg-perception/object_recognition_msgs.git -b ros2 \
    # && git clone https://github.com/OctoMap/octomap_msgs.git -b ros2
    # && for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done \
    # && sudo apt update --fix-missing \
    # && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y \

WORKDIR /home/$USERNAME/ws_chessaton
# COPY --chown=ros ./src src

COPY chessaton_description /home/$USERNAME/ws_chessaton/src/chessaton_description
COPY chessaton_moveit_config /home/$USERNAME/ws_chessaton/src/chessaton_moveit_config
COPY chessaton_interfaces /home/$USERNAME/ws_chessaton/src/chessaton_interfaces
COPY chessaton_control /home/$USERNAME/ws_chessaton/src/chessaton_control
COPY chessaton_chess_manager /home/$USERNAME/ws_chessaton/src/chessaton_chess_manager
COPY chessaton_chessaton_arm_ikfast_plugin /home/$USERNAME/ws_chessaton/src/chessaton_chessaton_arm_ikfast_plugin

# Copy code into workspace, run rosdep install for workspace, build, and source setup in ros user's


# installing moveit and other dependencies
RUN sudo apt install -y \
    ros-$ROS_DISTRO-moveit\
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-moveit-ros-perception \
    ros-$ROS_DISTRO-gazebo-ros2-control \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-gazebo-ros \
    ros-$ROS_DISTRO-object-recognition-msgs

# config and install gazebo grasp plugin
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rm -rf ./home/$USERNAME/ws_chessaton/install ./home/$USERNAME/ws_chessaton/build ./home/$USERNAME/ws_chessaton/log \
    && export rmw_implementation=rmw_cyclonedds_cpp \
    && sudo apt update --fix-missing \
    && mkdir /home/$USERNAME/ws_chessaton/src/gazebo-pkgs \
    && git clone https://github.com/kongoncharuk/gazebo-pkgs.git -b $ROS_DISTRO /home/$USERNAME/ws_chessaton/src/gazebo-pkgs \
    && sudo rosdep install --from-paths . --ignore-src -r -y --rosdistro=${ROS_DISTRO} \
    && colcon build --symlink-install --parallel-workers 1 --packages-select gazebo_version_helpers gazebo_grasp_plugin moveit_task_constructor_msgs rviz_marker_tools moveit_task_constructor_core

# install chessaton
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source ~/ws_chessaton/install/setup.bash \
    && colcon build --symlink-install --parallel-workers 1 --packages-select chessaton_description chessaton_moveit_config chessaton_control chessaton_chessaton_arm_ikfast_plugin chessaton_interfaces chessaton_chess_manager \
    && source ~/ws_chessaton/install/setup.bash \
    && echo 'source ~/ws_chessaton/install/local_setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /usr/share/gazebo/setup.bash' >> /home/$USERNAME/.bashrc \
    && export GAZEBO_PLUGIN_PATH=/usr/share/gazebo/../../lib/x86_64-linux-gnu/gazebo-11/plugins:/home/ros/workspace/build/gazebo_grasp_plugin \
    && sudo chown ros -R /home/$USERNAME/ws_chessaton/src \
    && /home/$USERNAME/ws_chessaton/src/chessaton_description/scripts/xacro2urdf.bash \
    && /home/$USERNAME/ws_chessaton/src/chessaton_description/scripts/xacro2sdf.bash \
    && /home/$USERNAME/ws_chessaton/src/chessaton_moveit_config/scripts/xacro2srdf.bash