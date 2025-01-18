# UR5e Meta Package

This is a meta package for the UR5e arm and dual RG2 grippers.

## Overview

This package provides the necessary configurations and launch files to operate the UR5e robotic arm equipped with dual RG2 grippers.

## Contents

- **ur5e_description**: Contains the URDF and meshes for the UR5e robot.
- **ur5e_moveit_config**: MoveIt configuration for the UR5e robot.
- **rg2_gripper_description**: Contains the URDF and meshes for the RG2 grippers.
- **ur5e_rg2_bringup**: Launch files to bring up the UR5e robot with RG2 grippers.

## Installation

Clone this repository into your catkin workspace and build the workspace:

```bash
cd ~/catkin_ws/src
git clone <repository_url>
cd ~/catkin_ws
catkin_make
```

## Usage

To launch the UR5e robot with dual RG2 grippers, use the following command:

```bash
roslaunch ur5e_rg2_bringup ur5e_rg2_bringup.launch
```

## Maintainers

- [Your Name](mailto:your.email@example.com)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.