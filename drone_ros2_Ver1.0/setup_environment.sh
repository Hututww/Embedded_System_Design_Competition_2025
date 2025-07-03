#!/bin/bash
# 瑞萨开发板Yolov5火焰检测ROS2项目环境配置脚本

# 1. 更新系统和安装依赖
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-pip python3-colcon-common-extensions \
    python3-rosdep ros-humble-desktop \
    build-essential cmake git libopencv-dev

# 2. 配置ROS2环境
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 3. 初始化rosdep
sudo rosdep init
rosdep update

# 4. 创建工作空间
mkdir -p ~/drone_ws/src
cd ~/drone_ws/src

# 5. 克隆Yolov5-ROS仓库
git clone https://github.com/doleron/yolov5-ros.git
cd yolov5-ros
git checkout humble  # 切换到Humble分支

# 6. 克隆自定义无人机控制包
git clone https://github.com/yourusername/drone_control.git

# 7. 安装Python依赖
cd ~/drone_ws/src/yolov5-ros/yolov5
pip3 install -r requirements.txt

# 8. 下载Yolov5火焰检测模型
wget https://github.com/ultralytics/yolov5/releases/download/v6.0/yolov5s.pt -O ~/drone_ws/src/yolov5-ros/yolov5/weights/yolov5s.pt

# 9. 编译工作空间
cd ~/drone_ws
colcon build --packages-select yolov5_ros drone_control
source install/setup.bash
echo "source ~/drone_ws/install/setup.bash" >> ~/.bashrc    