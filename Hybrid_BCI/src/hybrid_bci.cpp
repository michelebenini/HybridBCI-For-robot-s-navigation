#include "ros/ros.h"

#include "hybrid_bci/motorimagery.h"
#include "hybrid_bci/P300.h"
#include "hybrid_bci/P300_person.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "tf/tfMessage.h"
#include "angles/angles.h"

#include "iostream"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>



#include <hybrid_bci/ParametersConfig.h>

struct distribution                                       // struct represent normal distribution
{
  double *dir;
  double mean;
  double std_dev;
};

struct player                                             // struct represent player data
{
  int id;
  cv::Point2d pos;
  double p;
};

class bot                                                // struct represent the bot data
{
  public:
    cv::Point2d current_pos;
    int current_direction;
    int target_direction;
    struct distribution dir;
    int enable_moves;

    void p300Callback(const hybrid_bci::P300::ConstPtr& msg);
    void p300_send_one(player *pls,int n);
    void motorimageryCallback(const hybrid_bci::motorimagery::ConstPtr& msg);
    void motor_imagery_right();
    void motor_imagery_left();
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
};

void _init_distribution(struct distribution *dis, double mean, double std_dev, int n);
double normal_distribution_calc(int n, double mean, double std_dev);
void sum_distribution(double *out, double *in, int n);
void mul_distribution(double *out, double *in, int n);
void mul_distribution(double *out, double mul, int n);
void shift_distribution(double *out, int shift, int n);
double tot_distribution(double *in, int n);
int calculate_degree(cv::Point2d bt, cv::Point2d elem, int n);
double dist(cv::Point2d p1, cv::Point2d p2);
int calc_direction(double *in, int n);
cv::Point2d calculate_point(cv::Point2d bt, int dg, double dist);
void confirm_move(bot *bt);
geometry_msgs::Twist bot_move(int *change, bot *bt);

void bot::p300Callback(const hybrid_bci::P300::ConstPtr& msg)
{
  //ROS_INFO("[ %d ] P300 command!", ((int)msg->pkg_id));
  //ROS_INFO("START DIRECTION : %d",this->current_direction);
  int people = ((int)msg->tot_people);
  struct player pls[people];
  for(int i = 0; i < people; i++){
    //ROS_INFO("\t[ %d ] ( %d - %d ) %lf",(int)msg->person[i].id, (int)msg->person[i].x,(int)msg->person[i].y,(double)msg->person[i].p);
    pls[i].id = (int)msg->person[i].id;
    pls[i].pos = cv::Point2d((int)msg->person[i].x,(int)msg->person[i].y);
    pls[i].p = (double)msg->person[i].p;
  }
  p300_send_one(pls,people);
  ROS_INFO("DIRECTION : %d\n",this->target_direction);
}

void bot::p300_send_one(player *pls,int n){           // send the p300 command
  hybrid_bci::ParametersConfig config = hybrid_bci::ParametersConfig::__getDefault__();
  int max_direction = config.max_direction;
  //ROS_INFO("max_direction %d",max_direction);
  double p300_past_effect = config.p300_past_effect;
  //ROS_INFO("p300_past_effect %lf",p300_past_effect);
  int p300_stddev = config.p300_stddev;
  //ROS_INFO("p300_stddev %d",p300_stddev);
  
  double acc = 0;
  int max1 = (pls[0].p > pls[1].p) ? 0:1;
  int max2 = (pls[0].p > pls[1].p) ? 1:0;
  
  for(int i = 2; i < n; i++){
    if(pls[i].p > pls[max1].p){
      max2 = max1;
      max1 = i;
    }
    else if(pls[i].p > pls[max2].p){
      max2 = i;
    }
    
  }
  //ROS_INFO("MAX1 %lf MAX2 %lf ", pls[max1].p, pls[max2].p);
  mul_distribution(dir.dir, p300_past_effect, max_direction);
  //ROS_INFO("Scaled distribution %lf ",tot_distribution(dir.dir, max_direction));
    
  double ratio = 1 + exp(2*pls[max2].p/pls[max1].p);
  //ROS_INFO("ratio %lf",ratio);
  
  distribution k_dist;
  k_dist.dir = (double*)calloc(max_direction,sizeof(double));
  _init_distribution(&k_dist, (double)max_direction/2, p300_stddev*ratio, max_direction); // possibility to add a coefficient whitch depends of the players distance 
  //ROS_INFO("New distribution %lf ",tot_distribution(k_dist.dir, max_direction));
  
  mul_distribution(k_dist.dir, (1-p300_past_effect), max_direction);
  //ROS_INFO("Scaled new distribution %lf ",tot_distribution(k_dist.dir, max_direction));
  
  int shift = calculate_degree(current_pos, pls[max1].pos, max_direction);
  //ROS_INFO("shift %d ", shift);
  
  shift_distribution(k_dist.dir, shift, max_direction);
  sum_distribution(dir.dir, k_dist.dir, max_direction);

  double tot = tot_distribution(dir.dir, max_direction);
  mul_distribution(dir.dir, 1/tot, max_direction);
  this->target_direction = calc_direction(dir.dir, max_direction);
  //ROS_INFO("CURRENT DIR : %d ", this->target_direction);
  free(k_dist.dir);
}

void bot::motorimageryCallback(const hybrid_bci::motorimagery::ConstPtr& msg)
{
  hybrid_bci::ParametersConfig config = hybrid_bci::ParametersConfig::__getDefault__();
  int max_direction = config.max_direction;
  if(((int)msg->pkg_id) < 0){
    free(this->dir.dir);
    exit(0);
  }
  if(msg->dir){
    //ROS_INFO("[ %d ] Turn right!", ((int)msg->pkg_id));
    //ROS_INFO("START DIRECTION : %d",this->current_direction);
    motor_imagery_right();
    ROS_INFO("DIRECTION : %d",this->target_direction);
  }
  else if(!msg->dir){
    //ROS_INFO("[ %d ] Turn left!", ((int)msg->pkg_id));
    //ROS_INFO("START DIRECTION : %d",this->current_direction);
    motor_imagery_left();
    ROS_INFO("DIRECTION : %d",this->target_direction);
  }
  
}

void bot::motor_imagery_right(){                                          // send motor imagery right command
  //std::cout << "Right command!" << std::endl;
  hybrid_bci::ParametersConfig config = hybrid_bci::ParametersConfig::__getDefault__();
  int max_direction = config.max_direction;
  double mi_stddev = config.mi_stddev;
  double mi_past_effect = config.mi_past_effect;
  double new_stay_effect = config.new_stay_effect;
  int mi_degree = config.mi_degree;
  double stay_stddev = config.stay_stddev;
  distribution mi;
  mi.dir = (double*)calloc(max_direction,sizeof(double));
  _init_distribution(&mi, max_direction/2, mi_stddev, max_direction);
  mul_distribution(mi.dir, (1-mi_past_effect-new_stay_effect), max_direction);
  int shift = current_direction - max_direction/2 - mi_degree;
  shift_distribution(mi.dir, shift, max_direction);
  
  distribution k_dist;
  k_dist.dir = (double*)calloc(max_direction,sizeof(double));
  _init_distribution(&k_dist, max_direction/2, stay_stddev,max_direction);
  mul_distribution(k_dist.dir, new_stay_effect, max_direction);
  
  mul_distribution(dir.dir, mi_past_effect, max_direction);
  sum_distribution(dir.dir, mi.dir, max_direction);

  shift_distribution( k_dist.dir, (target_direction-max_direction/2), max_direction);
  sum_distribution(dir.dir, k_dist.dir, max_direction);
  
  double tot = tot_distribution(dir.dir, max_direction);
  mul_distribution(dir.dir, 1/tot, max_direction);
  this->target_direction = calc_direction(dir.dir, max_direction);
  free(mi.dir);
  free(k_dist.dir);
}

void bot::motor_imagery_left(){                                          // send motor imagery left command
  hybrid_bci::ParametersConfig config = hybrid_bci::ParametersConfig::__getDefault__();
  int max_direction = config.max_direction;
  double mi_stddev = config.mi_stddev;
  double mi_past_effect = config.mi_past_effect;
  double new_stay_effect = config.new_stay_effect;
  int mi_degree = config.mi_degree;
  double stay_stddev = config.stay_stddev;
  distribution mi;
  mi.dir = (double*)calloc(max_direction,sizeof(double));
  _init_distribution(&mi, (double)max_direction/2, mi_stddev, max_direction);
  mul_distribution(mi.dir, (1-mi_past_effect-new_stay_effect), max_direction);
  int shift = current_direction -max_direction/2 + mi_degree;
  shift_distribution(mi.dir, shift, max_direction);
  
  distribution k_dist;
  k_dist.dir = (double*)calloc(max_direction,sizeof(double));
  _init_distribution(&k_dist, (double)max_direction/2, stay_stddev, max_direction);
  mul_distribution(k_dist.dir, new_stay_effect, max_direction);
  
  mul_distribution(dir.dir, mi_past_effect, max_direction);
  sum_distribution(dir.dir, mi.dir, max_direction);

  shift_distribution( k_dist.dir, (target_direction-max_direction/2), max_direction);
  sum_distribution(dir.dir, k_dist.dir, max_direction);
  
  double tot = tot_distribution(dir.dir, max_direction);
  mul_distribution(dir.dir, 1/tot, max_direction);
  this->target_direction = calc_direction(dir.dir, max_direction);
  free(mi.dir);
  free(k_dist.dir);
}

void bot::odomCallback(const nav_msgs::Odometry::ConstPtr &msg){
   
  current_pos = cv::Point2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
  std::cout << current_pos << std::endl;
  double rad = tf::getYaw(msg->pose.pose.orientation);
  double deg = angles::to_degrees(rad);
  if(deg < 0) deg = deg + 360;
  std::cout << deg << std::endl;
  current_direction = (int)deg;

  if(target_direction == -1){
    target_direction = current_direction;
    _init_distribution(&dir,180,180, 360);
    shift_distribution( dir.dir, current_direction, 360);
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10000);
  bot bt;

  bt.current_pos = cv::Point2d(0,0);
  bt.target_direction = -1;
  bt.current_direction = 0;
  bt.dir.dir = (double*)calloc(360,sizeof(double));
  bt.enable_moves = 1;
  

  ros::Subscriber p300_sub = n.subscribe("p300", 1, &bot::p300Callback, &bt);
  ros::Subscriber mi_sub = n.subscribe("motorimagery", 1, &bot::motorimageryCallback, &bt);
  ros::Subscriber od_sub = n.subscribe("odom",1, &bot::odomCallback, &bt);
  ros::Rate loop_rate(1);
  
  int count = 0;
  int change = 0;
  while(ros::ok){
    geometry_msgs::Twist msg;
    if(count % 5 == 0 && change == 0){
      confirm_move(&bt);
    }
    if(change == 1){
      count = 1;
    }
    msg =  bot_move(&change, &bt);
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }
  

  return 0;
}

 void _init_distribution(struct distribution *dis, double mean, double std_dev, int n){
  // ROS_INFO("direction %d ", n);
  dis->mean = mean;
  dis->std_dev = std_dev;
  for(int i = 0; i < n; i++){
    dis->dir[i] = normal_distribution_calc(i, mean, std_dev); 
  }
}

double normal_distribution_calc(int n, double mean, double std_dev){
  double res = (1/sqrt(2*M_PI*std_dev*std_dev))*exp(-(n-mean)*(n-mean)/(2*std_dev*std_dev));
  return res;
}
  
void sum_distribution(double *out, double *in, int n){
  for(int i = 0; i < n; i++){
    out[i] = out[i] + in[i];
  }
}

void mul_distribution(double *out, double *in, int n){
  for(int i = 0; i < n; i++){
    out[i] = out[i] * in[i];
  }
}

void shift_distribution(double *out, int shift, int n){
  double in[n];
  for(int i = 0; i < n; i++){
    in[i] = out[i];
  }

  for(int i = 0; i < n; i++){
    int index = i-shift;
    if(index >= n)index = index-n;
    if(index < 0)index = n+index;
    out[i] = in[index];
    
  }
  
}

void mul_distribution(double *out, double mul, int n){
  if(mul < 0)return;
  for(int i = 0; i < n; i++){
    out[i] = out[i] * mul;
  }
}

double tot_distribution(double *in, int n){
  double res = 0;
  for(int i = 0; i < n; i++){
    res = res + in[i];
  }
  return res;
}

int calculate_degree(cv::Point2d bt, cv::Point2d elem, int n){                               // calculate degree beetwen two point2ds
  double d = dist(bt, elem);
  cv::Point2d pt0(0,d);
  elem = elem - bt;

  double dp = elem.x*pt0.x + elem.y*pt0.y ;
  double norm = d*sqrt(elem.x*elem.x + elem.y*elem.y);
  double cosin = dp / norm;
  double res = acos(cosin) * (180.0 / M_PI);
  
  if(elem.x > 0)res = 360-(res+180);
  if(elem.x < 0)res = res+180;
  
  res = res*n/360;
  int result = (((int)res)%360);
  result = (result >= 0) ? result:n+result;
  return result;
}

double dist(cv::Point2d p1, cv::Point2d p2){                                          // calculate the Euclidean distance beetwen two points
  return sqrt((p1.x - p2.x)*(p1.x - p2.x)+(p1.y - p2.y)*(p1.y - p2.y)) ;
}

int calc_direction(double *in, int n){                                                   // calculate the target direction basing on distribution
  double acc[n];
  for(int i = 0; i < n; i++){
    acc[i] = 0;
    for(int j = -1; j < 1; j++){
      int index = i + j;
      if(index < 0){
        index = n + index;
      }
      else if(index >= n){
        index = index - n;
      }
      acc[i] = acc[i] + in[index];
    }
  }
  int max_pos = 0;
  for(int i = 1; i < n; i++){
      if(acc[i] > acc[max_pos]){
        max_pos = i;
      }
  }
  return max_pos;
}

cv::Point2d calculate_point(cv::Point2d bt, int dg, double dist){                     // calculare the point at distance dist and degree dg from bt
  cv::Point2d pt;

  double angle = (double)dg*M_PI/180;
  double s = dist*sin(angle);
  double c = dist*cos(angle);

  pt.x = bt.x - (int)s;
  pt.y = bt.y + (int)c;

  return pt;
}

void confirm_move(bot *bt){
  hybrid_bci::ParametersConfig config = hybrid_bci::ParametersConfig::__getDefault__();
  int max_direction = config.max_direction;
  double move_effect = config.move_effect;
  int move_stddev = config.move_stddev;
  
  std::cout << "Move command!" << std::endl;
  int dis = bt->current_direction - bt->target_direction;
  if((dis >= - max_direction/2) && (dis < 0 || dis >= max_direction/2)){
    bt->current_direction++;
    bt->current_direction = bt->current_direction%360;
  }
  else if((dis <= max_direction/2) && (dis > 0 || dis <= -max_direction/2)){
    bt->current_direction--;
    if(bt->current_direction < 0){
      bt->current_direction = 360 - bt->current_direction;
    }
  }
  
  //bt->current_direction = bt->target_direction; // if the robot turn to the target direction without moving
  
 
  distribution move;
  move.dir = (double*)calloc(max_direction,sizeof(double));
  _init_distribution(&move, (double)max_direction/2, move_stddev, max_direction);
  mul_distribution(move.dir, move_effect,max_direction);
  int shift = (bt->target_direction-(max_direction/2));
  shift_distribution(move.dir, shift, max_direction);
  sum_distribution(bt->dir.dir, move.dir, max_direction);
  
  double tot = tot_distribution(bt->dir.dir, max_direction);
  mul_distribution(bt->dir.dir, 1/tot, max_direction);
  free(move.dir);

}

geometry_msgs::Twist bot_move(int *change, bot *bt){
  *change = 0;
  
  geometry_msgs::Twist msg;
  int range = 20;
  int dif = bt->current_direction - bt->target_direction;
  if(dif < 0) dif = dif + 360;
  if(dif > 359) dif = dif - 360;
  
  if(bt->enable_moves){
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
      *change = 1;
    }
  }

  return msg;
}