#include "ros/ros.h"
#include "hybrid_bci/direction_distribution.h"

#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

struct distribution                                       // struct represent normal distribution
{
  double *dir;
  double mean;
  double std_dev;
};

class bot                                                // struct represent the bot data
{
  public:
    cv::Point2d current_pos;
    int current_direction;
    int target_direction;
    struct distribution dir;
    int enable_moves;
    cv::Mat img;
    char *name;

    void showCallback(const hybrid_bci::direction_distribution::ConstPtr& msg);
    
};
void bot::showCallback(const hybrid_bci::direction_distribution::ConstPtr& msg){
    ROS_INFO("Callback show");
    img = cv::Mat(cv::Size(800, 800), CV_8UC3, cv::Scalar(255,255,255));
    for(int i = 0; i < 360; i++){
        dir.dir[i] = msg->directions[i];
    }
    cv::Scalar color(0,0,0);
    int radius_dist = (int)(img.rows-10)/2;
    int radius_area = 10;
    double max = -1;
    double coef_r = 0;
    double coef_c = 0;
    cv::Point pos = cv::Point(img.rows/2,img.cols/2);
    cv::circle(img, pos, radius_area, color, -1, CV_AA);
    cv::circle(img, pos, radius_dist, color, 1, CV_AA);
    
    for(int i = 0; i < 360; i++)
        if(max < dir.dir[i])
        max = dir.dir[i];
    coef_r = (radius_dist-radius_area)/max;  
    coef_c = 255/max; 
    ROS_INFO("Max r: %lf ",coef_r); 
    ROS_INFO("Max c: %lf ",coef_c); 
    ROS_INFO("Max %lf ",max); 
    for(int i = 0; i < 360; i++){
        color[1] = (int)(coef_c * dir.dir[i]);
        for(int j = radius_area; j < radius_dist && j < radius_area+(coef_r*dir.dir[i]); j++){
        cv::ellipse(img, pos, cv::Size(j,j), 0.0, (double)i+(360/4) , (double)(i + 1)+(360/4), color, 2, CV_AA, 0 );
        }  
    }
    cv::imshow(name, img);
    char q = cv::waitKey(1);
    if(q =='q'){
      cv::destroyWindow(name);
      exit(0);
    }
    
}

int main(int argc, char **argv){
  ros::init(argc, argv, "view_dist");
  ros::NodeHandle n;
  bot bt;
  bt.dir.dir = (double*)calloc(360,sizeof(double));
  bt.enable_moves = -1;
  bt.name = (char*)"Direction distribution";
  
  cv::namedWindow(bt.name, CV_WINDOW_NORMAL);

  ros::Subscriber show_sub = n.subscribe("move", 1, &bot::showCallback, &bt);
  
  ros::spin();
  cv::destroyWindow(bt.name);
  
  free(bt.dir.dir);
  return 0;
}