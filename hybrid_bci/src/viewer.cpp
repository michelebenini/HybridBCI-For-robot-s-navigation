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

struct person_roi
{
  int id;
  cv::Point corner;
  int height;
  int width;
  int active;
  
};

class view{
  public:
    int full_screen;
    double mi_left;
    double mi_right;
    int start;
    person_roi *pls;
    int n_person;
    cv::Mat img;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void mi_rawCallback(const hybrid_bci::motorimagery_raw::ConstPtr& msg);
    void p300_roiCallback(const hybrid_bci::P300_roi::ConstPtr& msg);
};

void view::p300_roiCallback(const hybrid_bci::P300_roi::ConstPtr& msg){
  n_person = msg->tot_people;
  if(n_person > 0){
    pls = (person_roi*)calloc(n_person, sizeof(person_roi));
    for(int i = 0; i < n_person; i++){
      pls[i].id = msg->person[i].id;
      pls[i].height = msg->person[i].height;
      pls[i].width = msg->person[i].width;
      pls[i].active = msg->person[i].active;
      pls[i].corner = cv::Point(msg->person[i].y,msg->person[i].x);
    }
  }
}

void view::mi_rawCallback(const hybrid_bci::motorimagery_raw::ConstPtr& msg){
  mi_left = msg->left;
  mi_right = msg->right;
}

void view::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  char str[200];
  try
  {
    cv::Mat cam = cv_bridge::toCvShare(msg, "bgr8")->image;
    int bar_rows = 50;
    int start_rows = 50;
    img = cv::Mat(cam.rows+bar_rows+start_rows, cam.cols, CV_8UC3);
    
    int lf_start = (1-mi_left)*(cam.cols/2 - 2);
    int rg_end = cam.cols/2 + 2 + mi_right*(cam.cols/2 - 2);
    int bar = (cam.cols)/20;
    
    for(int i = 0; i < img.rows; i++){
      for(int j = 0; j < img.cols; j++){
        if(i < start_rows){
          img.at<cv::Vec3b>(cv::Point(j,i)) = cv::Vec3b(255,255,255);
        }
        else if(i < cam.rows+start_rows){
          img.at<cv::Vec3b>(cv::Point(j,i)) = cam.at<cv::Vec3b>(cv::Point(j,i-start_rows));
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
    
    int act = 0;
    for(int i = 0; i < n_person; i++){
      cv::Rect rect = cv::Rect(pls[i].corner.x + start_rows,pls[i].corner.y,pls[i].width,pls[i].height);
      
      if(pls[i].active == 1){
        act = 1;
        cv::Mat roi = img(rect);
        roi.setTo(cv::Scalar(0, 255, 0));
      }
      else{
        
        cv::rectangle(img, rect, cv::Scalar(0, 0, 0));
      }
    }

    if(act == 0 && n_person >= 0){
      cv::Mat roi = img(cv::Rect(0,0,img.cols, start_rows));
      roi.setTo(cv::Scalar(0, 255, 0));
    }

    sprintf(str,"START/STOP");
    putText(img, str, cv::Point((img.cols/2)-70,35), cv::FONT_HERSHEY_DUPLEX, 1,  cv::Scalar(0,0,0), 2, cv::LINE_AA);
    
    if(full_screen)cv::setWindowProperty("Viewer", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN); // FULL SCREEN
    
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
    view v;
    v.full_screen = 0;
    v.mi_left = 0;
    v.mi_right = 0;
    v.start = 0;
    v.n_person = -1;

    cv::namedWindow("Viewer", CV_WINDOW_NORMAL);
    image_transport::ImageTransport it(n);
    image_transport::Subscriber img_sub = it.subscribe("camera/rgb/image_raw", 1, &view::imageCallback, &v);
    ros::Subscriber mi_sub = n.subscribe("mi_raw",1, &view::mi_rawCallback, &v);
    ros::Subscriber p300_sub = n.subscribe("p300_roi",1, &view::p300_roiCallback, &v);
    ros::spin();
    cv::destroyWindow("Viewer");
    free(v.pls);
}