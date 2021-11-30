#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
	ros::init(argc, argv, "simple_navigation_goals");
	
	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);
	
	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");  
	}
	
	move_base_msgs::MoveBaseGoal goal;
	
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();
	
	goal.target_pose.pose.position.x = 7.0;
	goal.target_pose.pose.position.y = 1.0;
	goal.target_pose.pose.position.z = 0.0;
	goal.target_pose.pose.orientation.z = 1.0;
	
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	ac.waitForResult(ros::Duration(50.0));
	ac.cancelAllGoals();
	
	goal.target_pose.pose.position.x = 0.0;
	goal.target_pose.pose.position.y = 0.0;
	goal.target_pose.pose.position.z = 0.0;
	goal.target_pose.pose.orientation.z = 1.0;

	ROS_INFO("Going back");
	ac.sendGoal(goal);
	ac.waitForResult();
	ac.cancelAllGoals();

	ROS_INFO("The base moved and now is (kinda) back");
	
	return 0;
}
