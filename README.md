# Teach and Repeat using Bézier Curves
This repository implements a **Teach and Repeat** (T&R) navigation system for mobile robots, allowing them to autonomously follow previously demonstrated paths.
<!-- TODO: Add a better description-->

## Dependencies
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html);
- Ubuntu 22.04;
- Colcon Common Extensions;
- Turtlebot3 simulation;

## Methods
There are two methods to follow the demonstrated path:

- **Dot-to-dot (nodes/repeat_path_coords.py)**: The robot follows a Pure Pursuit-like approach where it intercepts a look-ahead point. Once it reaches this point, a new point further along the path is set as the next target.
- **Bezier curve based (nodes/repeat_bezier_path.py)**: The robot simulates multiple potential paths ahead using Bézier curves and selects the optimal path based on predefined criteria.

## Installation
To set up the Teach and Repeat system, follow these steps:

Clone this repository along with its submodules:
  ```zsh
  git clone --recurse-submodules https://github.com/jardeldyonisio/teach_and_repeat.git
  ```

Make the installation script executable and run it with superuser privileges:
  ```zsh
  chmod +x install.sh
  sudo ./install.sh
  ```

Initialize `rosdep` in your workspace:
  ```zsh
  rosdep init
  ```

Update `rosdep` to fetch the latest package information:
  ```zsh
  rosdep update
  ```

Install the required ROS package dependencies:
  ```zsh
  rosdep install --from-paths src -y --ignore-src
  ```

## Run

After setting up the environment, follow these steps to run the system:

### Demonstrate Path

To demonstrate a path, follow these steps:

Launch the navigation system:
```zsh
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
**You must use 'Pose2DEstimation' to define the initial position.**

Open another terminal and start the teleoperation node:
```zsh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Control the robot using the following keys:
- `I` to move forward
- `K` to move backward
- `J` to turn left
- `L` to turn right
- `Q` to increase speed
- `Z` to decrease speed
- `Space` to stop

Start the path demonstration:
```zsh
ros2 launch lognav_navigation teach_path_coords
```
To end the demonstration the user have to press CTRL + C.

### Follow Path

To follow a path, first ensure that the coordinates from the path demonstration are stored in the `teach_and_repeat/data/teleop_data.txt` file. This file is also the default file read during the path following process. Make sure you start from the same point where the demonstration began.

If the navigation system is not already running, launch it:
```zsh
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Then, run the path following node:
```zsh
ros2 run teach_and_repeat repeat_bezier_path
```

**After starting the repeat node, set a pose using '2DPoseEstimate'. The robot will immediately begin following the path.**

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
