FROM elainasuki/ros:ros2-jazzy-full

# 定义用户和用户组
ARG USERNAME=Elaina
WORKDIR /home/$USERNAME

RUN apt-get update && apt-get install -y ros-jazzy-robot-localization ros-jazzy-slam-toolbox ros-jazzy-pointcloud-to-laserscan ros-jazzy-nav2-bringup
USER root
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> /home/$USERNAME/.bashrc \
    && echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> /home/$USERNAME/.config/fish/config.fish \
    && echo "source ~/visual/install/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source ~/packages/ros_ws/install/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "bass source ~/packages/ros_ws/install/setup.bash" >> /home/$USERNAME/.config/fish/config.fish \
    && echo "bass source ~/visual/install/setup.bash" >> /home/$USERNAME/.config/fish/config.fish 

CMD ["/bin/bash"]

