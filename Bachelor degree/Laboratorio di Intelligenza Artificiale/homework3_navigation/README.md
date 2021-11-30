####

To command the robot from the keyboard, install teleop-twist-keyboard via:
  sudo apt-get install ros-<distro>-teleop-twist-keyboard

documentation at http://wiki.ros.org/teleop_twist_keyboard

The terminal tab that runs the teleop node must be active during operation. To this extent, you must split the screen to visualize the stage simulation while commanding the robot.

####

Gmapping documentation http://wiki.ros.org/gmapping 

####

To install map-server run:
  sudo apt install ros-<distro>-map-server

documentation at http://wiki.ros.org/map_server

You need the map saver once gmapping had processed all the data in your bag.Map server instead is used to provide to both the localizer and the planner the resulting map.

###

Specific prompt commands I used:

  (se serve, source ./devel/setup.bash)

  roscd stage_ros
  >>>rosrun stage_ros stageros world/willow-erratic.world

  rosrun teleop_twist_keyboard teleop_twist_keyboard.py

  rosbag record --all -o hw3.bag

  giretto ijkl col robot

  rosbag play --pause hw3_2020-04-29-18-15-50.bag

  rosrun gmapping slam_gmapping scan:=base_scan

  rosrun map_server map_saver -f hw3_map

  .yaml e .pgm creati

  >>>rosrun map_server map_server hw3_map.yaml

  >>>rosrun thin_navigation thin_localizer_node

  >>>rosrun thin_navigation thin_planner_node

  >>>rviz
  in rviz add map, particlecloud, path, base_pose_ground_truth

  pos estimate e nav goal.


