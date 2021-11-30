#include <ros/ros.h>
#include <turtlesim/GetCircles.h>
#include <turtlesim/SpawnCircle.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/RemoveCircle.h>
#include <turtlesim/Kill.h>
#include <turtlesim/Circle.h>
#include <turtlesim/Pose.h>

turtlesim::Pose g_pose;

void poseCallback(const turtlesim::Pose pose)
{
  ros::NodeHandle n;
  
  //richiede servizi
  ros::ServiceClient client1 = n.serviceClient<turtlesim::RemoveCircle>("remove_circle");
  ros::ServiceClient client2 = n.serviceClient<turtlesim::GetCircles>("get_circles");
  turtlesim::RemoveCircle srv1;
  turtlesim::GetCircles srv2;
  
  //confronta coordinate principale-secondarie e chiama la delete
  if(client2.call(srv2)){ 
	for (int i = 0; i < srv2.response.circles.size(); i++){
		g_pose = pose;
		float c1 = srv2.response.circles[i].x - g_pose.x, c2 = srv2.response.circles[i].y - g_pose.y;
		if (c1 < 0.30 && c2 < 0.30 && c1 > -0.30 && c2 > -0.30){
			  srv1.request.id = srv2.response.circles[i].id;
			  client1.call(srv1);
		}	
	}
  }
}

int main(int argc, char **argv)
{
  if (argc != 1){
	ROS_INFO("usage: remove_circle_client");
    return 1;
  }
  
  ros::init(argc, argv, "remove_circle_client");
  ros::NodeHandle n;  
  
  //si iscrive per ricevere Pose
  ros::Subscriber pose_sub = n.subscribe<turtlesim::Pose>("turtle1/pose", 10, poseCallback);
  ros::spin();
  
  ROS_ERROR("Failed to call service remove_circle");
  return 0;
}

