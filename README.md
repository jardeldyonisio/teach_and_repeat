# Teach and Repeat

This repository implements a **teach-and-repeat** navigation system for mobile robots, enabling them to follow previously taught paths autonomously.

## Features
- Supports path teaching and repetition using odometry and IMU, or a combination of map, odometry, and IMU.
- Two path representations: dot-to-dot and Bézier curves.
- Available for differential and tricycle-type robots.
- The vehicle stops when an obstacle is detected below a defined threshold.

## Dependencies
- ROS 2 Humble
- Ubuntu 22.04
- robot_localization (for Extended Kalman Filter)

## Installation
1. Clone this repository:

```
git clone https://github.com/jardeldyonisio/teach_and_repeat.git
```


## Usage
To teach a path, ensure you run a teleop node to control the robot along the desired path. To finalize the teaching process, press `CTRL + C` to stop recording the positions:
```
ros2 run teach_and_repeat teach_path_coords.py
```
To repeat a path using dot-to-dot method:
```
ros2 launch teach_and_repeat repeat_bezier_path.launch.py
```
To repeat a path using bezier curve method:
```
ros2 launch teach_and_repeat repeat_path_coords.launch.py
```

## TODO
- Create a ".yaml" file to configure:
  - Define the robot model;
  - Define lookahead parameters.
- Use more than one Bézier curve to represent the path (B-splines).
- Create launch files to define paths.

## Demo Videos
Check out the teach and repeat system in action:

- **Teach and Repeat using Odometry and Map**: [Watch the demo on YouTube](https://www.youtube.com/watch?v=7bjSsaD-_tI)
- **Obstacle Detection**: [Watch the obstacle detection demo on YouTube](https://www.youtube.com/watch?v=RmauNqaVmGg)
- **Teach and Repeat using Odometry Only**: [Watch the demo on YouTube](https://www.youtube.com/watch?v=-z7Gqplbi0U&t=5s)

## Contributing
Feel free to open issues or submit pull requests to improve this project.
