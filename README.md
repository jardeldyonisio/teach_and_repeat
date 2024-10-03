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

## Installation
1. Clone this repository:
   ```
   git clone https://github.com/jardeldyonisio/teach_and_repeat.git
   ```

## TODO

- Create a ".yaml" file to configure:
  - Define the robot model;
  - Define lookahead params.
- Use more than one bezier curve to represent the path (bsplines);
- Create launch files:
  - Define path

## Demo Videos
Check out the teach and repeat system in action:

- **Teach and Repeat using Odometry and Map**: [Watch the demo on YouTube](https://www.youtube.com/watch?v=7bjSsaD-_tI)
- **Obstacle Detection**: [Watch the obstacle detection demo on YouTube](https://www.youtube.com/watch?v=RmauNqaVmGg)
- **Teach and Repeat using Odometry Only**: [Watch the demo on YouTube](https://www.youtube.com/watch?v=-z7Gqplbi0U&t=5s)


