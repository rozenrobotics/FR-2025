FROM ros:noetic-ros-base-focal

RUN echo 'Etc/UTC' > /etc/timezone && \
    rm -rf /etc/localtime && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    build-essential \
    dh-make \
    cmake \
    gnupg \
    curl \
    git \
    python3-pip \
    python3-rosdep \
    ros-noetic-catkin \
    xmlstarlet \
    && rm -rf /var/lib/apt/lists/*

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

SHELL ["/bin/bash", "-c"]

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    source /opt/ros/noetic/setup.bash && \
    mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws && \
    catkin_make && \
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
    source ~/.bashrc

RUN cd ~/catkin_ws/src && \
    git clone --depth 1 https://github.com/CopterExpress/clover && \
    git clone --depth 1 https://github.com/CopterExpress/ros_led && \
    git clone --depth 1 https://github.com/ethz-asl/mav_comm
    
RUN cd ~/catkin_ws/ && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -y

RUN cd ~/catkin_ws/src && \
    git clone --recurse-submodules https://gitlab.com/Artemy2807/fr-team-task-rozen && \
    /usr/bin/python3 -m pip install -r ~/catkin_ws/src/clover/clover/requirements.txt && \
    /usr/bin/python3 -m pip install -r ~/catkin_ws/src/fr-team-task-rozen/offboard/app/requirements.txt

RUN git clone --recursive --depth 1 --branch v1.12.3 https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot && \
    ln -s ~/PX4-Autopilot ~/catkin_ws/src/ && \
    ln -s ~/PX4-Autopilot/Tools/sitl_gazebo ~/catkin_ws/src/ && \
    ln -s ~/PX4-Autopilot/mavlink ~/catkin_ws/src/

RUN cd ~/catkin_ws/src/PX4-Autopilot/Tools/setup && \
    ./ubuntu.sh && \
    pip3 install --user toml && \
    ln -s ~/catkin_ws/src/clover/clover_simulation/airframes/* ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/ && \
    /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

RUN cd ~/catkin_ws && \
    source /opt/ros/noetic/setup.bash && \
    catkin_make -j10 && \
    ./src/fr-team-task-rozen/setup.bash

CMD ["bash"]
