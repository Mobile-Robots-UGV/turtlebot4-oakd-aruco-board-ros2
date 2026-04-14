
# turtlebot4-oakd-aruco-board-ros2

ROS 2 package for TurtleBot 4 using the OAK-D camera to detect a 4-marker ArUco board and publish the board pose as:

- `x, y, z`
- `rx, ry, rz` (roll, pitch, yaw)

This repository contains a ROS 2 package for board pose estimation from the TurtleBot 4 OAK-D camera stream, along with calibration artifacts used to estimate pose accurately. The node subscribes to the robot camera image topic, detects a configured ArUco board, estimates its 6-DoF pose using camera calibration + board geometry, and publishes position, orientation, visibility, detected marker IDs, and TF.

## Acknowledgment and prior work

This repository is based on prior work from:

- [`Mobile-Robots-UGV/oak-aruco-board-pose-ros2`](https://github.com/Mobile-Robots-UGV/oak-aruco-board-pose-ros2)

That upstream repository provides the original board-pose estimation framework, calibration workflow, and board-configuration approach for an OAK camera. This TurtleBot 4 repository adapts that workflow to a ROS 2 deployment where the OAK-D camera stream is already available through the robot topic interface, and the pose node operates directly from ROS image topics instead of a direct `depthai` device pipeline. The upstream repository documents the same core outputs: `/board_pose`, `/board_rpy`, `/board_visible`, `/board_used_ids`, and TF from camera frame to board frame.

## What this repository does

This package detects a known 4-marker ArUco board mounted in the environment or on a target object and estimates the board pose relative to the robot camera.

Given:

- a calibrated OAK-D camera
- a known ArUco dictionary
- a known board geometry
- the robot image stream

the node computes:

- board position in camera frame: `x, y, z` in meters
- board orientation in camera frame: `rx, ry, rz`
- visibility status
- detected marker IDs used for the pose solution
- a TF transform from the camera frame to the board frame

This makes the package useful for:

- target localization
- follower / tracker pipelines
- relative pose feedback
- debugging marker visibility
- downstream state estimation and control
- RViz / TF inspection

## Repository contents

At the top level, this repository currently contains:

- `camera_calib_images/`
- `ros2_ws/src/board_pose_ros/`
- `README.md`
- `LICENSE` 

### `camera_calib_images/`

This directory contains files used during camera calibration, such as:

- captured calibration images
- calibration scripts
- the resulting camera calibration file

In our workflow, the final calibration file is:

- `camera_calib_oak.npz`

This file contains the camera intrinsics and distortion parameters used by the pose node.

### `ros2_ws/src/board_pose_ros/`

This is the ROS 2 package. It contains the runtime node, configuration files, and launch files needed to detect the ArUco board and publish pose.

Typical contents include:

- the Python node implementation
- `config/board_config.json`
- `config/camera_calib_oak.npz`
- launch file(s)
- ROS 2 package metadata

## System overview

### Input

The TurtleBot 4 publishes camera topics from the OAK-D sensor. In this project, the pose node is configured to subscribe to the compressed RGB topic:

- `/robot_09/oakd/rgb/image_raw/compressed`

This is consistent with the TurtleBot 4 / OAK-D topic layout where the robot exposes RGB image topics and compressed transport variants through ROS 2. 

### Processing pipeline

The runtime flow is:

1. Subscribe to the compressed camera image topic.
2. Decode the compressed image.
3. Detect ArUco markers in the selected dictionary.
4. Match the detected markers against the configured board layout.
5. Assemble 2D image points and 3D board points.
6. Solve the board pose with OpenCV PnP.
7. Convert the estimated rotation to roll, pitch, yaw.
8. Publish ROS topics and TF.

### Output

The package publishes:

- `geometry_msgs/PoseStamped` on `/board_pose`
- `geometry_msgs/Vector3Stamped` on `/board_rpy`
- `std_msgs/Bool` on `/board_visible`
- `std_msgs/Int32MultiArray` on `/board_used_ids`
- TF transform from camera frame to board frame

These outputs are the same core interface documented in the upstream repository. 

## ROS topics

### Subscribed topics

Primary subscription:

- `/robot_09/oakd/rgb/image_raw/compressed`  
  Type: `sensor_msgs/msg/CompressedImage`

This is the compressed RGB image stream used for marker detection and pose estimation.

Depending on your deployment, the topic name may differ by robot namespace. Update the launch file or node parameter accordingly.

### Published topics

#### `/board_pose`
Type: `geometry_msgs/msg/PoseStamped`

Publishes the estimated 6-DoF board pose in the camera frame.

Contents:
- `position.x`
- `position.y`
- `position.z`
- quaternion orientation

Interpretation:
- `x`: horizontal displacement of the board relative to the camera
- `y`: vertical displacement of the board relative to the camera
- `z`: forward distance from camera to board

#### `/board_rpy`
Type: `geometry_msgs/msg/Vector3Stamped`

Publishes Euler angles derived from the pose rotation matrix:

- `vector.x = roll`
- `vector.y = pitch`
- `vector.z = yaw`

Units are typically radians unless explicitly converted in the node logs.

#### `/board_visible`
Type: `std_msgs/msg/Bool`

Publishes whether a valid board pose is currently available.

- `true`: board visible and pose solved
- `false`: board not detected or pose not solvable

#### `/board_used_ids`
Type: `std_msgs/msg/Int32MultiArray`

Publishes the detected marker IDs that were actually used for pose estimation.

This is useful for:
- debugging incomplete detections
- verifying board geometry
- checking whether the expected markers are visible

### TF output

The node broadcasts a transform from the camera frame to the board frame.

Typical frame relationship:

- parent: `oak_camera_frame`
- child: `board_frame`

This allows RViz and TF tools to visualize the estimated board pose in the TF tree.

## Board configuration

The board is defined in a JSON configuration file, typically:

- `config/board_config.json`

The configuration specifies:

- board size
- marker size
- ArUco dictionary
- marker IDs
- marker top-left coordinates in board frame
- per-marker rotation if needed

The upstream repository uses a 4-marker example board config with marker IDs `1, 3, 2, 4` and includes support for per-marker rotations in the board definition. This repository follows the same board-description concept. 

A typical board config contains:
- `board_size_m`
- `marker_size_m`
- `dictionary`
- `markers`

Each marker entry defines where that marker sits on the board in metric coordinates.

## Camera calibration

Accurate pose estimation requires a valid camera calibration file. In this repository, calibration is stored as:

- `config/camera_calib_oak.npz`

This file contains:
- `camera_matrix`
- `dist_coeffs`

Our calibration workflow used captured board images and produced a valid calibration file with acceptable reprojection error. The upstream repository also expects a `.npz` calibration file with camera intrinsics and distortion coefficients for runtime pose estimation. 

## Requirements

### Software

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3
- OpenCV with ArUco support
- NumPy
- `rclpy`
- `tf2_ros`

The upstream repository states it was developed for ROS 2 Jazzy on Ubuntu and uses Python, launch files, parameters, topics, and TF2 in the same way.

### Robot / hardware

- TurtleBot 4
- OAK-D camera stream available over ROS 2
- printed and mounted 4-marker ArUco board

## Installation

### 1. Create a workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Mobile-Robots-UGV/turtlebot4-oakd-aruco-board-ros2.git
````

### 2. Install dependencies

```bash
sudo apt update
sudo apt install python3-opencv python3-numpy
python3 -m pip install --user --break-system-packages opencv-contrib-python numpy
```

If your environment already provides OpenCV ArUco support, the pip install may not be necessary.

### 3. Build the package

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select board_pose_ros --symlink-install
source ~/ros2_ws/install/setup.bash
```

This follows the same `colcon build --packages-select board_pose_ros --symlink-install` build pattern documented by the upstream repository. 

## How to run

### 1. Make sure the robot camera topic is available

Confirm that the robot is publishing the compressed image topic:

```bash
ros2 topic list | grep /robot_09/oakd/rgb/image_raw/compressed
```

### 2. Source the workspace

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 3. Launch the board pose node

```bash
ros2 launch board_pose_ros board_pose.launch.py
```

## Runtime behavior

When the board is visible, the node logs pose estimates such as:

```text
visible=True ids=[1, 4, 3, 2] x=0.0199 y=-0.0337 z=0.7311 rx=-2.9851 ry=-0.1904 rz=-0.0256
```

In this example:

* the board is visible
* the detected board markers include IDs `1, 4, 3, 2`
* the board center is approximately:

  * `x = 0.0199 m`
  * `y = -0.0337 m`
  * `z = 0.7311 m`
* the board orientation is:

  * `rx = -2.9851`
  * `ry = -0.1904`
  * `rz = -0.0256`

These values change continuously as the board moves relative to the camera.

## How to inspect outputs

### List board topics

```bash
ros2 topic list | grep board
```

### Echo visibility

```bash
ros2 topic echo /board_visible
```

### Echo used marker IDs

```bash
ros2 topic echo /board_used_ids
```

### Echo board pose

```bash
ros2 topic echo /board_pose
```

### Echo roll, pitch, yaw

```bash
ros2 topic echo /board_rpy
```

### Inspect TF

```bash
ros2 run tf2_ros tf2_echo oak_camera_frame board_frame
```

These are the same runtime checks recommended by the upstream repository for validating visibility, marker IDs, pose, and TF.

## What the output looks like

When the system is working correctly, you should observe:

* `/board_visible` switching to `true`
* `/board_used_ids` containing the visible board marker IDs
* `/board_pose` publishing a valid `PoseStamped`
* `/board_rpy` publishing `roll, pitch, yaw`
* TF showing a transform from the camera frame to the board frame
* console logs showing stable `x y z rx ry rz`

Expected qualitative behavior:

* `z` increases as the board moves farther away
* `x` changes as the board moves left/right in the image
* `y` changes as the board moves up/down in the image
* `rx, ry, rz` change as the board tilts or rotates

## Example use cases

This package can be used for:

* board-based localization experiments
* robot-to-target relative pose estimation
* visual docking or alignment tasks
* object-of-interest tracking with a known fiducial target
* debugging camera calibration and board configuration
* feeding perception data into motion-control pipelines

## Calibration workflow used in this project

This repository also includes calibration-related files under `camera_calib_images/`. The calibration process used:

1. live image visualization from the robot camera
2. manual capture of only valid ChArUco views
3. offline calibration to produce `camera_calib_oak.npz`
4. use of that calibration file inside the ROS 2 package

This is conceptually aligned with the upstream repository, which includes both calibration support and runtime board tracking tooling.

## Differences from the upstream repository

Compared with `oak-aruco-board-pose-ros2`, this repository is adapted for TurtleBot 4 deployment:

* uses the TurtleBot 4 OAK-D ROS image stream
* subscribes to a ROS compressed image topic
* does not require direct camera access through a standalone OAK `depthai` pipeline for runtime detection
* is organized around a TurtleBot 4 ROS workflow
* retains the same board-pose outputs and general estimation logic

The upstream repository explicitly describes OAK camera input via `depthai`, while this TurtleBot 4 repository is aimed at operation from ROS topics already published by the robot.

## Known assumptions

This package assumes:

* the OAK-D camera is already publishing images
* the camera calibration file matches the active camera stream
* the board configuration matches the real printed board
* marker IDs, marker size, and board geometry are correct
* the ArUco dictionary in the config matches the printed markers

If any of these are wrong, the pose can be unstable, scaled incorrectly, or fail entirely.

## Troubleshooting

### The node launches but `/board_visible` stays false

Check:

* the image topic name is correct
* the board is actually in view
* the dictionary matches the printed markers
* the board config marker IDs are correct
* OpenCV ArUco support is installed

### Pose exists but scale is wrong

Check:

* `marker_size_m`
* board coordinates in `board_config.json`
* the calibration file
* whether the calibration corresponds to the same camera stream used at runtime

### IDs are detected but pose is unstable

Check:

* lighting and glare
* motion blur
* partial occlusion
* incorrect per-marker rotation in config
* inaccurate camera calibration

### Topic name is different on another robot

Update the image topic parameter in the launch file or node parameters to the correct namespace.



