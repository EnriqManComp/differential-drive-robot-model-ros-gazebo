# Differential Drive Robot (DDR) model for using in ROS and Gazebo
## Commands
### XACRO file to URDF file 
```cmd 
xacro two_wheeled_robot.xacro > two_wheeled_robot_without_xacro.urdf
```
### URDF file to SDF file 
```cmd
gz sdf -p two_wheeled_robot_without_xacro.urdf > two_wheeled_robot.sdf
```
### Package for the control of the robot
```cmd
sudo apt-get install ros-noetic-teleop-twist-keyboard
```
### Command for launch the gazebo with an empty world
```cmd
roslaunch gazebo_ros empty.world
```

