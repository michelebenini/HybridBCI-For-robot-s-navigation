#include "ros/ros.h"
#include "iostream"

#include <image_transport/image_transport.h>
#include <hybrid_bci/motorimagery_raw.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "hybrid_bci/P300_roi.h"
#include "hybrid_bci/person_roi.h"


//#include "cnbiros_bci/TidInterface.hpp"

class view{
  public:
    int full_screen;
    double mi_left;
    double mi_right;
    int start;
    cv::Mat img;
    void mi_rawCallback(const hybrid_bci::motorimagery_raw::ConstPtr& msg);
    void image();
};



void view::mi_rawCallback(const hybrid_bci::motorimagery_raw::ConstPtr& msg){
  mi_left = msg->left;
  mi_right = msg->right;
}

void view::image(){
  char str[200];

  int bar_rows = 50;
  int start_rows = 0;
  img = cv::Mat(bar_rows, 500, CV_8UC3);
  
  int lf_start = (1-mi_left)*(img.cols/2 - 2);
  int rg_end = img.cols/2 + 2 + mi_right*(img.cols/2 - 2);
  int bar = (img.cols)/20;
  
  for(int i = 0; i < img.rows; i++){
    for(int j = 0; j < img.cols; j++){
      
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
  
  
  cv::imshow("Bars", img);
  char q = cv::waitKey(1);
  if(q =='q'){
    cv::destroyWindow("Bars");
    exit(0);
  }

}


int main(int argc, char **argv){
    ros::init(argc, argv, "Bars");
    ros::NodeHandle n;
    view v;
    v.full_screen = 0;
    v.mi_left = 0;
    v.mi_right = 0;


    cv::namedWindow("Bars", CV_WINDOW_NORMAL);
    
    ros::Subscriber mi_sub = n.subscribe("mi_raw",1, &view::mi_rawCallback, &v);
    ros::Rate loop_rate(1);

    while(ros::ok){
      v.image();
      ros::spinOnce();
      loop_rate.sleep();
    }
    cv::destroyWindow("Bars");
}