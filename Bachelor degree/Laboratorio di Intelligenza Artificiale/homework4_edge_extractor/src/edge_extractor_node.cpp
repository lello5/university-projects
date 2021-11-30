#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

void imageCallback(const sensor_msgs::CompressedImage& msg)
{
	cv_bridge::CvImagePtr original = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	cv_bridge::CvImagePtr res = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	cv::threshold(res->image, res->image, 127, 255, CV_THRESH_BINARY);
	cv::GaussianBlur(res->image, res->image, cv::Size(3, 3), 0, 0);
	cv::Canny(res->image, res->image, 70, 210, 3);
	std::string or_name = "originale", res_name = "risultato";
	cv::imshow(or_name, original->image);
	cv::imshow(res_name, res->image);
	cv::waitKey(2);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "edge_extractor_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/default/camera_node/image/compressed", 10, imageCallback);
  ros::spin();
}
