#安装相关语言环境
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

#增加ROS2 服务器解析地址
sudo sed -i '$a\ ' /etc/hosts
sudo sed -i  '$a\185.199.108.133 raw.githubusercontent.com' /etc/hosts
sudo sed -i  '$a\185.199.109.133 raw.githubusercontent.com' /etc/hosts
sudo sed -i  '$a\185.199.110.133 raw.githubusercontent.com' /etc/hosts
sudo sed -i  '$a\185.199.111.133 raw.githubusercontent.com' /etc/hosts

#安装必要的工具软件
sudo apt update && sudo apt install curl -y

#安装ROS2软件密钥
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

#更新软件列表，安装ROS2 桌面版本
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-desktop  -y

#安装ROS2 机器人必要的功能模块
sudo apt-get install ros-humble-joint-* -y
sudo apt-get install libasio-dev -y
sudo apt-get install ros-humble-visualization-msgs* \
                     ros-humble-geo* ros-humble-sensor-msgs*  \
                     ros-humble-nav* ros-humble-rclcpp* \
                     ros-humble-std-* ros-humble-tf2* \
                     ros-humble-laser-* ros-humble-pcl-* \
                     ros-humble-message-* ros-humble-rosidl-* \
                     ros-humble-rcutils*  ros-humble-rosbag2-*

#单次配置ROS2缓存，提升性能
sudo sysctl net.ipv4.ipfrag_high_thresh=134217728
sudo sysctl -w net.core.rmem_max=2147483647