#include <ros/ros.h>
#include <turtlesim/GetCircles.h>
#include <turtlesim/SpawnCircle.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/RemoveCircle.h>
#include <turtlesim/Kill.h>
#include <turtlesim/Circle.h>
#include <turtlesim/Pose.h>

std::vector <turtlesim::Circle> circles;
int id = 2;

bool spawn_circle(turtlesim::SpawnCircle::Request &req, turtlesim::SpawnCircle::Response &res)
{
  ROS_INFO("request: x=%f, y=%f", (float)req.x, (float)req.y);
  ROS_INFO("spawning a circle in those coords..");
  
  //crea tartaruga
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  srv.request.x = (float) req.x;
  srv.request.y = (float) req.y;
  srv.request.theta = 0.0;
  client.call(srv);
  
  //aggiorna circles
  turtlesim::Circle newcircle;
  newcircle.id = id++;
  newcircle.x = (float) req.x;
  newcircle.y = (float) req.y;
  circles.push_back(newcircle);
  res.circles = circles;
  
  return true;
}

bool get_circles(turtlesim::GetCircles::Request &req, turtlesim::GetCircles::Response &res)
{
  res.circles = circles;
  return true;
}

bool remove_circle(turtlesim::RemoveCircle::Request &req, turtlesim::RemoveCircle::Response &res)
{
  ROS_INFO("removing circle %d", req.id);
  for (int i = 0; i < circles.size(); i++){
	  if(circles[i].id == req.id){
		  //aggiorna circles
		  circles.erase(circles.begin()+i);
		  
		  //elimina tartaruga
		  ros::NodeHandle n;
		  ros::ServiceClient client = n.serviceClient<turtlesim::Kill>("kill");
		  turtlesim::Kill srv;
		  char buffer[10];
		  int retval = sprintf(buffer, "turtle%d", req.id);
		  srv.request.name = buffer;
		  client.call(srv);
	  }
  }
  res.circles = circles;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spawn_circle_server");
  ros::NodeHandle n;
  
  //pubblicizza servizi
  ros::ServiceServer service1 = n.advertiseService("spawn_circle", spawn_circle);
  ROS_INFO("Ready to spawn circles.");
  ros::ServiceServer service2 = n.advertiseService("get_circles", get_circles);
  ROS_INFO("Ready to get circles.");
  ros::ServiceServer service3 = n.advertiseService("remove_circle", remove_circle);
  ROS_INFO("Ready to remove circles.");
  
  ros::spin();

  return 0;
}
