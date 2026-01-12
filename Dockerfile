# 使用 ROS 2 Humble 基础镜像
FROM osrf/ros:humble-desktop

# 设置环境变量
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

# 安装 MoveIt 2 和 Panda 相关的依赖
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-task-constructor-core \
    ros-humble-moveit-task-constructor-planning \
    ros-humble-moveit-task-constructor-loader \
    ros-humble-moveit-task-constructor-visualization \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    git \
    && rm -rf /var/lib/apt/lists/*

# 创建工作空间
WORKDIR /root/ros2_ws

# 复制源代码 (假设你的代码在 src 目录下)
COPY ./src ./src

# 安装 Python 依赖 (如果有 requirements.txt)
# COPY requirements.txt .
# RUN pip3 install -r requirements.txt

# 解决依赖并编译
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# 设置启动命令 (Source 环境)
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc
