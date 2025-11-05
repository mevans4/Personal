# Lidar Tree Detector

## Build the Package

```sh
cd ~/41068_ws
colcon build --symlink-install
```

## Source the Workspace

```sh
source ~/41068_ws/install/setup.bash
```
## Run the world file

```sh
ros2 launch 41068_ignition_bringup 41068_ignition_drone.launch.py rviz:=true world:=PlantationTest
```

## Run the Tree Detector Node

```sh
ros2 run lidar_tree_detector lidar_tree_detector_node
```

## View Detected Trees in the Terminal

```sh
ros2 topic echo /detected_trees
```


