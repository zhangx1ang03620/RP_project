# Jazzy server
FROM osrf/ros:jazzy-desktop

# Set up a non-interactive front end
ENV DEBIAN_FRONTEND=noninteractive

# Install the dependency libraries required by the project. 
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

# Install code quality tools
COPY requirements.txt .
RUN pip3 install -r requirements.txt --break-system-packages

# Set up workspace
WORKDIR /root/ros2_ws
COPY ./src ./src

# Compilation
# Note: The path to Jazzy's setup.sh is also in the same pattern.
RUN . /opt/ros/jazzy/setup.sh && colcon build

# Set environment variables
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc
