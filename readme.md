# Teach and Repeat

This package uses the *teach and repeat* technique to teach a certain path while the operator drives the logistics vehicle and then repeats the path autonomously. See below how you can test the package.

## Dependencies
- Ubuntu;
- [ROS Humble;](https://docs.ros.org/en/humble/Installation.html)

## Preparing the package

Create your workspace:
```
mkdir -p ~/colcon_ws/src
```

Access the folder:
```
cd ~/colcon_ws/src
```

Clone this package:
```
git clone --recurse-submodules --remote-submodules https://github.com/jardeldyonisio/teach_and_repeat
```

Compile the package:
```
colcon build
```

## Steps to test

In the freedom_navigation package, ensure that the branch used is main:
```
git checkout main
```

To start the world and spawn the vehicle:
```
ros2 launch freedom_vehicle simple_vehicle.launch.py
```

To teach the path use the following command:
```
ros2 run freedom_vehicle teach_path_coords.py
```

To repeat the path using the dot-to-dot method:
```
ros2 run freedom_vehicle repeat_path_coords.py
```

To repeat the path using the Bézier curve-based method:
```
ros2 run freedom_vehicle repeat_bezier_path.py
```
