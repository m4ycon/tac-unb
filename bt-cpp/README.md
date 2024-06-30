--- comandos comuns

source /opt/ros/humble/setup.zsh

ros2 pkg create --build-type ament_python --node-name my_node_name my_package_name


ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true rviz:=true nav2:=true

ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true

source install/setup.zsh

colcon build
cmake . && make
ros2 launch rome_bt execute_bt.launch.py bt:="./behavior_trees/deliverSample.xml"


--- problemas em ver o scan no rviz

cd /opt/ros/humble/share/irobot_create_description/urdf/
sudo code create3.urdf.xacro
# substitui ogre por ogre2

---

ros2 run nav2_map_server map_saver_cli -f "my_map"

---

sudo apt install ros-$ROS_DISTRO-turtle-tf2-py ros-$ROS_DISTRO-tf2-tools ros-$ROS_DISTRO-tf-transformations

sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install ignition-fortress
