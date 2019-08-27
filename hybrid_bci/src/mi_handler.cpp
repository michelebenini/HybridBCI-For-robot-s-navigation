#include "ros/ros.h"
#include "iostream"
#include <thread>         // std::thread
#include <mutex>          // std::mutex

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include "hybrid_bci/motorimagery.h"


#include "cnbiros_tobi_msgs/TicMessage.h"
#include "cnbiros_tobi_msgs/TicClassifier.h"
#include "cnbiros_tobi_msgs/TicClass.h"
#include <hybrid_bci/ParametersConfig.h>

class view{
  public:
    int full_screen;
    double mi_left;
    double mi_right;
    int start;
    int count_mi;
    cv::Mat img;
    ros::Publisher pubcmd;

    void cnbCallback(const cnbiros_tobi_msgs::TicMessage::ConstPtr& msg);
    void image();
};

std::mutex mtx_bars;


void view::cnbCallback(const cnbiros_tobi_msgs::TicMessage::ConstPtr& msg){
  ROS_INFO("SMR callback");
  hybrid_bci::ParametersConfig config = hybrid_bci::ParametersConfig::__getDefault__();
  
  double left = -1;
  double right = -1;

  std::string label0 = msg->classifiers[0].classes[0].label;
  std::string label1 = msg->classifiers[0].classes[1].label;
  
  if(label0.compare(config.mi_cmd_left) == 0 && label1.compare(config.mi_cmd_right) == 0){
    left = msg->classifiers[0].classes[0].value;
    right = msg->classifiers[0].classes[1].value;
  }
  else if(label1.compare(config.mi_cmd_left) == 0 && label0.compare(config.mi_cmd_right) == 0){
    right = msg->classifiers[0].classes[0].value;
    left = msg->classifiers[0].classes[1].value;
  }
  else{
    ROS_INFO("ERROR in motor imagery command");
    return;
  }

  while(!mtx_bars.try_lock());
    mi_left = mi_left + left;
    mi_right = mi_right + right;
    double sum = mi_left+mi_right;
    mi_left = mi_left/sum;
    mi_right = mi_right/sum;
  mtx_bars.unlock();

  ROS_INFO("Right command: %lf", mi_right);
  ROS_INFO("Left command: %lf", mi_left);

  if(mi_left > config.mi_threshold){
    ROS_INFO("Send Left command!");
    hybrid_bci::motorimagery msg;
      msg.pkg_id = count_mi;
      msg.dir = true;
      pubcmd.publish(msg);
      count_mi++;
  }
  else if(mi_right > config.mi_threshold){
    ROS_INFO("Send Right command!");
    hybrid_bci::motorimagery msg;
      msg.pkg_id = count_mi;
      msg.dir = true;
      pubcmd.publish(msg);
      count_mi++;
  }

}

void view::image(){
  char str[200];

  int bar_rows = 50;
  img = cv::Mat(bar_rows, 500, CV_8UC3);
  
  while(!mtx_bars.try_lock());
    int lf_start = (1-mi_left)*(img.cols/2 - 2);
    int rg_end = img.cols/2 + 2 + mi_right*(img.cols/2 - 2);
    int bar = (img.cols)/20;
  mtx_bars.unlock();

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
    v.count_mi = 0;


    cv::namedWindow("Bars", CV_WINDOW_NORMAL);
    v.pubcmd = n.advertise<hybrid_bci::motorimagery>("motorimagery", 10000);
    ros::Subscriber mi_sub = n.subscribe("rostic_cnbi2ros",1, &view::cnbCallback, &v);
    ros::Rate loop_rate(1);

    while(ros::ok){
      v.image();
      ros::spinOnce();
      loop_rate.sleep();
    }
    cv::destroyWindow("Bars");
}