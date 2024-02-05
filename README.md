<h1>Differential Drive Robot (DDR) model for using in ROS and Gazebo</h1>
<h2> Commands </h2>
xacro two_wheeled_robot.urdf > two_wheeled_robot_without_xacro.urdf<br><br>
gz sdf -p two_wheeled_robot_without_xacro.urdf > two_wheeled_robot.sdf<br><br>
roslaunch gazebo_ros empty.world<br><br>
sudo apt-get install ros-noetic-teleop-twist-keyboard<br><br>
