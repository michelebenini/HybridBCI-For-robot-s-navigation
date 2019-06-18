#include "ros/ros.h"
#include "iostream"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("Viewer", cv_bridge::toCvShare(msg, "bgr8")->image);
    char q = cv::waitKey(1);
    if(q =='q'){
      cv::destroyWindow("Viewer");
      exit(0);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "viewer");
    ros::NodeHandle n;

    cv::namedWindow("Viewer");
    image_transport::ImageTransport it(n);
    image_transport::Subscriber img_sub = it.subscribe("camera/rgb/image_raw", 1, imageCallback);
    ros::spin();
    cv::destroyWindow("Viewer");
}