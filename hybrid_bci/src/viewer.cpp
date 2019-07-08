#include "ros/ros.h"
#include "iostream"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#define MI_left 0.7
#define MI_right 0.2

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  int bar_high = 50;
  try
  {
    cv::Mat cam = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat img(cam.rows+50, cam.cols, CV_8UC3);
    int lf_start = (1-MI_left)*(cam.cols/2 - 2);
    int rg_end = cam.cols/2 + 2 + MI_right*(cam.cols/2 - 2);
    int bar = (cam.cols)/20;
    
    for(int i = 0; i < img.rows; i++){
      for(int j = 0; j < img.cols; j++){
        if(i < cam.rows){
          img.at<cv::Vec3b>(cv::Point(j,i)) = cam.at<cv::Vec3b>(cv::Point(j,i));
        }else{
          if(j <= img.cols/2 + 2 && j >= img.cols/2 - 2){
            img.at<cv::Vec3b>(cv::Point(j,i)) = cv::Vec3b(0,0,255);
          }
          else if( j % bar == 0){
            img.at<cv::Vec3b>(cv::Point(j,i)) = cv::Vec3b(0,0,0);
          }
          else if(j > lf_start && j < img.cols/2 - 2){
            img.at<cv::Vec3b>(cv::Point(j,i)) = cv::Vec3b(0,255,0);
          }
          else if(j > img.cols/2 + 2 && j < rg_end){
            img.at<cv::Vec3b>(cv::Point(j,i)) = cv::Vec3b(0,255,0);
          }
          else{
            img.at<cv::Vec3b>(cv::Point(j,i)) = cv::Vec3b(255,255,255);
          }
        }
      }
    }    
    //cv::setWindowProperty("Viewer", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN); // FULL SCREEN
    cv::imshow("Viewer", img);
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

    cv::namedWindow("Viewer", CV_WINDOW_NORMAL);
    image_transport::ImageTransport it(n);
    image_transport::Subscriber img_sub = it.subscribe("camera/rgb/image_raw", 1, imageCallback);
    ros::spin();
    cv::destroyWindow("Viewer");
}