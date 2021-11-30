#############################################################################################

in:
	cd workspaces/ws_mio

compilare:
	catkin build

eseguire:
	(source ./devel/setup.bash in ogni terminale)

	roscore
	rosrun turtlesim turtlesim_node
	rosrun turtlesim turtle_teleop_key
	rosrun turtlesim spawn_circle_server
	rosrun turtlesim spawn_circle_client N
	rosrun turtlesim get_circles_client
	rosrun turtlesim remove_circle_client
 
#############################################################################################
