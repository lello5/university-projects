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
  srand((unsigned) time(0));
  
  //terminale con o senza parametri (senza => N = 1)
  int N = 1;
  if (argc != 1 && argc != 2){
	ROS_INFO("usage: spawn_circle_client N");
    return 1;
  }
  if(argc == 2)
    N = atoll(argv[1]);
  
  ros::init(argc, argv, "spawn_circle_client");
  ros::NodeHandle n;
  
  //richiede servizio
  ros::ServiceClient client = n.serviceClient<turtlesim::SpawnCircle>("spawn_circle");
  turtlesim::SpawnCircle srv;
  
  //richiede spawn di N tartarughe
  for(int i = 0; i < N; i++) {
	  srv.request.x = (float (rand())/float((RAND_MAX)) * 9.0) +1.0;
	  srv.request.y = (float (rand())/float((RAND_MAX)) * 9.0) +1.0;
	  if (client.call(srv))
		ROS_INFO("spawned a circle in %f %f.", (float) srv.request.x, (float) srv.request.y);
	  else{
		ROS_ERROR("Failed to call service spawn_circle");
		return 1;
	  }
  }
  return 0;
}

