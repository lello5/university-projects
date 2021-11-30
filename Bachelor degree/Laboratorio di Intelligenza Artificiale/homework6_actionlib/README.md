Prompt commands:

(se serve, source ./devel/setup.bash)
roscore

roscd stage_ros
rosrun stage_ros stageros world/willow-erratic.world

cd /workspaces/HW6
rosrun map_server map_server hw3_map.yaml

cd /workspaces/labaigi_ws
rosrun thin_navigation thin_localizer_node
rosrun thin_navigation thin_planner_node

rviz
in rviz add map, particlecloud, path, base_pose_ground_truth

pos estimate
OPPURE
rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" 

cd /workspaces/HW6
rosrun hw6_actionlib SimpleClient2_node
