# Jazzy 的基础镜像
FROM osrf/ros:jazzy-desktop

# 设置非交互前端
ENV DEBIAN_FRONTEND=noninteractive

# 安装项目所需的依赖库 
RUN apt-get update && apt-get install -y \
    ros-jazzy-moveit \
    ros-jazzy-moveit-task-constructor-core \
    ros-jazzy-moveit-task-constructor-planning \
    ros-jazzy-moveit-task-constructor-loader \
    ros-jazzy-moveit-task-constructor-visualization \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-moveit-resources-panda-moveit-config \
    git \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 安装代码质量工具
COPY requirements.txt .
RUN pip3 install -r requirements.txt --break-system-packages

# 设置工作空间
WORKDIR /root/ros2_ws
COPY ./src ./src

# 编译
# 注意：Jazzy 的 setup.sh 路径也是一样的模式
RUN . /opt/ros/jazzy/setup.sh && colcon build

# 设置环境变量
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc
