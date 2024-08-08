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
git clone --recurse-submodules --remote-submodules https://github.com/lognav4-0/freedom_navigation
```

Also, clone the package freedom_vehicle in the src folder:
```
https://github.com/lognav4-0/freedom_vehicle
```

Compile the package:
```
colcon build
```

## Steps to test

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
