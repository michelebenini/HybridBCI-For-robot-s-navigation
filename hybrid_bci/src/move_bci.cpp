#include "ros/ros.h"
#include "hybrid_bci/direction_distribution.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "tf/tfMessage.h"
#include "angles/angles.h"

#include "iostream"

#include <image_transport/image_transport.h>
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

    void distCallback(const hybrid_bci::direction_distribution::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
};
void bot::odomCallback(const nav_msgs::Odometry::ConstPtr &msg){
   
  current_pos = cv::Point2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
  std::cout << current_pos << std::endl;
  double rad = tf::getYaw(msg->pose.pose.orientation);
  double deg = angles::to_degrees(rad);
  if(deg < 0) deg = deg + 360;
  std::cout << deg << std::endl;
  current_direction = (int)deg;
}
void bot::distCallback(const hybrid_bci::direction_distribution::ConstPtr& msg)
{
  target_direction = msg->best_dir;
  
  if(target_direction == -1){
    enable_moves = -1;
  }
  else{
    enable_moves = 1;
  } 
}

geometry_msgs::Twist bot_move(bot *bt);

int main(int argc, char **argv){
  ros::init(argc, argv, "move");
  ros::NodeHandle n;
  bot bt;
  bt.dir.dir = (double*)calloc(360,sizeof(double));
  bt.enable_moves = -1;
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10000);
  ros::Subscriber dist_sub = n.subscribe("move", 1, &bot::distCallback, &bt);
  ros::Subscriber od_sub = n.subscribe("odom",1, &bot::odomCallback, &bt);
  
  ros::Rate loop_rate(1);
  
  while(ros::ok){
    geometry_msgs::Twist msg;
    msg =  bot_move(&bt);
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  free(bt.dir.dir);
  return 0;
}

geometry_msgs::Twist bot_move(bot *bt){
 
  geometry_msgs::Twist msg;
  
  
  if(bt->enable_moves == 1){
    int range = 20;
    int dif = bt->current_direction - bt->target_direction;
    if(dif < 0) dif = dif + 360;
    if(dif > 359) dif = dif - 360;
    if(dif < range){
      ROS_INFO("Robot Moves straight");
      msg.linear.x = 0.4;
      msg.linear.y = 0;
      msg.linear.z = 0;
      msg.angular.x = 0;
      msg.angular.y = 0;
      msg.angular.z = 0;


    }
    else{
      
      ROS_INFO("Robot Moves angularly");
      
      msg.linear.x = 0;
      msg.linear.y = 0;
      msg.linear.z = 0;
      msg.angular.x = 0;
      msg.angular.y = 0;
      if((dif >= - 360/2) && (dif < 0 || dif >= 360/2)){
        msg.angular.z = 0.3;
      }
      else if((dif <= 360/2) && (dif > 0 || dif <= -360/2)){
        msg.angular.z = -0.3;
      }
      else{
        msg.angular.z = 0;
      }
    }
  }
  else{
      ROS_INFO("Robot Stops");
      msg.linear.x = 0;
      msg.linear.y = 0;
      msg.linear.z = 0;
      msg.angular.x = 0;
      msg.angular.y = 0;
      msg.angular.z = 0;
  }

  return msg;
}