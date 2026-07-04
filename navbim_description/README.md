# navbim_description

The `navbim_description` package contains URDF/Xacro robot description files for robots used with the navbim navigation stack.

## Usage

The robot description is loaded and published by the robot state publisher as part of the navbim bringup:

```bash
ros2 launch navbim_bringup robot_launch.py robot_name:=IDOG
```

To add your own robot, place a URDF/Xacro file in `urdf/<your_robot>.urdf.xacro` and pass `robot_name:=<your_robot>` to the launch file.
