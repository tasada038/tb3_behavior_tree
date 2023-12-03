# tb3_behavior_tree

This is the ROS 2 package to control Turtlebot3 with Behavior Tree.

![Build ROS2](https://github.com/tasada038/tb3_behavior_tree/actions/workflows/build.yml/badge.svg))

## Supported ROS 2 distributions

[![humble][humble-badge]][humble]
[![ubuntu22][ubuntu22-badge]][ubuntu22]

## Requirement

### Setting up the Turtlebot3 environment.

Please refer to the following page for installation.

[ROS2: Turtlebot3のGazeboシミュレーションをHumble/Noeticで動かす](https://zenn.dev/tasada038/articles/0a69eb6c6b444f)

```shell: Terminal
sudo nano ~/.bashrc
export TURTLEBOT3_MODEL=big_wheel
```

### Install BehaviorTree.CPP 3.8.x & Groot 1.0

Please refer to the following page for installation.

[ROS2: BehaviorTreeのチュートリアルを試す](https://zenn.dev/tasada038/articles/b7d193b567b94a)

## Build
```shell: Terminal
cd ~/dev_ws
colcon build --packages-select tb3_behavior_tree
```

## Usage
```shell: First terminal
cd ~/BehaviorTree/Groot/build/
./Groot
```


```shell: Second terminal
cd ~/turtlebot3_ws/
. install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_jp_world_empty.launch.py
```

```shell: terminal
cd ~/dev_ws/
. install/setup.bash
ros2 run tb3_behavior_tree tb3_behavior_node
```

## License
This repository is licensed under the MIT license, see LICENSE.

[humble-badge]: https://img.shields.io/badge/-HUMBLE-orange?style=flat-square&logo=ros
[humble]: https://docs.ros.org/en/humble/index.html

[ubuntu22-badge]: https://img.shields.io/badge/-UBUNTU%2022%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu22]: https://releases.ubuntu.com/jammy/
