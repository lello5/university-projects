#include <ros/ros.h>
#include <turtlesim/GetCircles.h>
#include <turtlesim/SpawnCircle.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/RemoveCircle.h>
#include <turtlesim/Kill.h>
#include <turtlesim/Circle.h>
#include <turtlesim/Pose.h>

int main(int argc, char **argv)
{
  if (argc != 1){
	ROS_INFO("usage: get_circles_client");
    return 1;
  }
  
  ros::init(argc, argv, "get_circles_client");
  ros::NodeHandle n;
  
  //richiede servizio
  ros::ServiceClient client = n.serviceClient<turtlesim::GetCircles>("get_circles");
  turtlesim::GetCircles srv;
  
  //stampa di circles returnato
  if (client.call(srv)){
	  for (int i = 0; i < srv.response.circles.size(); i++){
		  ROS_INFO("Circle %d in %f %f.", (int) srv.response.circles[i].id, (float) srv.response.circles[i].x, (float) srv.response.circles[i].y);
	  }
  }
  else{
	ROS_ERROR("Failed to call service get_circles");
	return 1;
  }
  
  return 0;
}

