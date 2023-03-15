# camera_module
A repository for ROS2 camera packages.

## ROS2 Install
- `cd ~`
- `source /opt/ros/{ros-version}/setup.bash`
- `mkdir -p ~/camera_ws/src`
- `cd ~/camera_ws/src`
- `git clone https://github.com/AD-SDL/camera_module.git`
- `cd ~/camera_ws`
- `sudo apt install python3-rosdep2`
- `rosdep update && rosdep install -i --from-path src`
- `sudo apt install python3-colcon-common-extensions`
- `colcon build`
- `source install/setup.bash`

## ROS2 Launch
Launching the publisher
- `ros2 launch camera_module_client camera_publisher.launch.py`
Launching the subscriber
- `ros2 launch camera_module_client camera_subscriber.launch.py`
### ROS2 Launch with Launch Parameters
Launching the publisher
- `ros2 launch camera_module_client camera_publisher.launch.py camera_name:=camera_module camera_number:=1`
- Camera name refers to the node name which will also be used in the topic name.
- Camera number refers to the device number 
Launching the subscriber
- `ros2 launch camera_module_client camera_subscriber.launch.py camera_name:=camera_module`
- In order to subscribe to the correct camera publisher, corresponding node name should be entered to "camera_name" parameter
