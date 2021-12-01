# ROS2 on ubuntu 20.04 with all packages installed
FROM tiryoh/ros2-desktop-vnc:galactic

RUN apt update && apt upgrade -y

# Install ROS 2 - GALACTIC
COPY install_ros2_galactic.sh .
RUN bash ./install_ros2_galactic.sh

## Install required packages
# Install Gazebo
RUN curl -sSL http://get.gazebosim.org | sh
RUN apt install -y ros-galactic-gazebo-*

# Install Cartographer
RUN apt install ros-galactic-cartographer -y
RUN apt install ros-galactic-cartographer-ros -y

# Install Navigation2
RUN apt install ros-galactic-navigation2 -y
RUN apt install ros-galactic-nav2-bringup -y

# Install cartographer
RUN apt install ros-galactic-cartographer -y
RUN apt install ros-galactic-cartographer-ros -y

# Install tools
RUN apt install python3-vcstool -y
RUN apt install ros-galactic-realtime-tools -y
RUN apt install ros-galactic-xacro -y

# Install control packages
RUN apt install ros-galactic-control-msgs -y
RUN apt install ros-galactic-control-toolbox -y
RUN apt install ros-galactic-gazebo-ros2-control -y
RUN apt install ros-galactic-hardware-interface* -y
RUN apt install ros-galactic-controller* -y

# Install python packages
RUN apt install pip -y
RUN pip install tensorflow
RUN pip install cloudpickle==1.2.0
RUN pip install gym
RUN pip install opencv-python
RUN pip install stable_baselines3

# Install extra tools
RUN apt install gedit -y
RUN apt install nano -y
RUN apt install htop -y

# Install TurtleBot3 from github
RUN /bin/bash -c 'mkdir $HOME/agrobot_ws && mkdir $HOME/agrobot_ws/src'
RUN /bin/bash -c '. /opt/ros/galactic/setup.bash; cd agrobot_ws/; colcon build --symlink-install'
COPY agrobot.repos $HOME/agrobot_ws/src
RUN cd $HOME/agrobot_ws/src && vcs import < agrobot.repos
RUN /bin/bash -c '. /opt/ros/galactic/setup.bash; cd agrobot_ws/; colcon build --symlink-install'

# Install agrobot packages
COPY agrobot $HOME/agrobot_ws/src/agrobot
RUN /bin/bash -c '. /opt/ros/galactic/setup.bash; cd ~/agrobot_ws/; colcon build --symlink-install'
RUN /bin/bash -c 'mkdir -p $HOME/.gazebo/models'
RUN cp -a $HOME/agrobot_ws/src/agrobot/agrobot_description/models/. $HOME/.gazebo/models/
RUN cd $HOME/agrobot_ws/src/agrobot/agrobot_rl/ && pip install -e gym-agrobot


COPY utils/desktop-items/Gazebo_logo.png /opt/gazebo/Gazebo_logo.png
COPY utils/desktop-items/controller_logo.png /opt/turtlebot3/controller_logo.png
COPY utils/desktop-items/moveit_logo.png /opt/turtlebot3/moveit_logo.png
COPY utils/desktop-items/openai_logo.png /opt/turtlebot3/openai_logo.png
COPY utils/desktop-items/rviz2.desktop ./Desktop/rviz2.desktop
COPY utils/desktop-items/simulation_base.desktop $HOME/Desktop/simulation_base.desktop
COPY utils/desktop-items/simulation_full.desktop $HOME/Desktop/simulation_full.desktop
COPY utils/desktop-items/start_rl.desktop $HOME/Desktop/start_rl.desktop
RUN cp /usr/share/applications/lxterminal.desktop $HOME/Desktop/lxterminal.desktop

RUN /bin/bash -c 'chmod +x $HOME/Desktop/simulation_full.desktop'
RUN /bin/bash -c 'chmod +x $HOME/Desktop/simulation_base.desktop'
# COPY utils/desktop-items/controller.desktop ./Desktop/
# COPY utils/desktop-items/moveit.desktop ./Desktop/
# COPY utils/desktop-items/manipulator_rviz.desktop ./Desktop/
