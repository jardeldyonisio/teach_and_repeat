# Teach and Repeat using Bézier Curves

This repository implements a **Teach and Repeat** (T&R) navigation system for mobile robots, allowing them to autonomously follow previously taught paths.
<!-- TODO: Add a better description-->

## TODO
- Create a `.yaml` file to configure:
  - The robot model;
  - Future behavior parameters.
- Use multiple Bézier curves (e.g., B-splines) to represent the path.
- Update the teach node to:
  - Accept a string input to name the path;
  - Require a key press to start collecting poses.
- Display poses during the teaching process.
- Show the circular threshold around the closest Bézier reference point.
- Automatically calculate `start_num_knots` based on the path.

## Features
- Supports relative (odometry-only, "odom" frame) and map-based teaching.
- Two path representations: dot-to-dot and Bézier curves.
- Compatible with differential and tricycle-type robots.
- Automatically stops the vehicle when an obstacle is detected within a defined threshold.

## Dependencies
- ROS 2 Humble
- Ubuntu 22.04

## Installation
1. Clone this repository:

```
git clone --recurse-submodules https://github.com/jardeldyonisio/teach_and_repeat.git
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

## Demo Videos
Check out the teach and repeat system in action:

- **Following a virtual path (Repeting process)**: [Watch the demo on YouTube](https://www.youtube.com/watch?v=crmZR9EUTow)
- **Teach and Repeat using Odometry and Map (Simulation)**: [Watch the demo on YouTube](https://www.youtube.com/watch?v=7bjSsaD-_tI)
- **Obstacle Detection (Simulation)**: [Watch the obstacle detection demo on YouTube](https://www.youtube.com/watch?v=RmauNqaVmGg)
- **Teach and Repeat using Odometry Only (Simulation)**: [Watch the demo on YouTube](https://www.youtube.com/watch?v=-z7Gqplbi0U&t=5s)

## Contributing
Feel free to open issues or submit pull requests to improve this project.

## Publication
For more details, refer to our publication [Teach and Repeat for Path Planning Using Bézier Curves](https://ieeexplore.ieee.org/document/10837801).

```
@INPROCEEDINGS{Dyonisio2024,
  author={J. Dos Santos Dyonisio and others},
  title={Teach and Repeat for Path Planning Using Bézier Curves},
  booktitle={2024 Brazilian Symposium on Robotics (SBR) and 2024 Workshop on Robotics in Education (WRE)},
  year={2024},
  pages={79-84},
  doi={10.1109/SBR/WRE63066.2024.10837801}
}

```
