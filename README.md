# ros2_sub_demo
Demo repository containing rosbag &amp; basic ROS2 node that subscribes to its contents

How to run:

Create ros2 workspace

```bash
mkdir -p ws/src && cd ws && git clone https://github.com/Serafadam/ros2_sub_demo.git
```

install dependencies

```bash
# in `ws` directory
sudo rosdep init
rosdep update 
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

build
```bash
# still in ws directory
colcon build --symlink-install && . install/
```

play rosbag
```bash
ros2 bag play -l src/ros2_sub_demo/rosbag2_2021_07_04-13_52_07/
```

run the node
```bash
ros2 run ros2_sub_demo test
```
you can also run the python file inside you IDE