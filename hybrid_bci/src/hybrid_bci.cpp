#include "ros/ros.h"

#include "hybrid_bci/motorimagery.h"
#include "hybrid_bci/direction_distribution.h"
#include "hybrid_bci/P300.h"
#include "hybrid_bci/P300_person.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "tf/tfMessage.h"
#include "angles/angles.h"
#include "std_msgs/Float64MultiArray.h"
#include <thread>         // std::thread
#include <mutex>          // std::mutex
#include "iostream"
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>


#include <hybrid_bci/ParametersConfig.h>

char* log_filename;
std::mutex mtx;

struct distribution                                       // struct represent normal distribution
{
  double *dir;
  double mean;
  double std_dev;
};

struct player                                             // struct represent player data
{
  int id;
  int dir;
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
hybrid_bci::direction_distribution bot_move( bot *bt);

void write_log(char* msg){
  FILE * f;
  
  auto tm = std::chrono::system_clock::now();
  std::time_t t = std::chrono::system_clock::to_time_t(tm);
  std::tm * ptm = std::localtime(&t);
  char buffer[32];
  // Format: Mo, 15.06.2009 20:20:00
  std::strftime(buffer, 32, "%a, %d.%m.%Y %H:%M:%S", ptm); 
  //std::cout << "Writing "<< log_filename <<": "<< msg << std::endl;
  while(!mtx.try_lock()) {};
      f = fopen(log_filename, "a");
      fprintf(f, "[%s] %s\n",buffer, msg);
      fclose(f);
  mtx.unlock();
}

void bot::p300Callback(const hybrid_bci::P300::ConstPtr& msg)
{
  //ROS_INFO("[ %d ] P300 command!", ((int)msg->pkg_id));
  ROS_INFO("START DIRECTION : %d",this->current_direction);
  hybrid_bci::ParametersConfig config = hybrid_bci::ParametersConfig::__getDefault__();
  int people = ((int)msg->tot_people);
  int max_direction = config.max_direction;
  ROS_INFO("People %d", people);
  if(people == -1 && enable_moves == 1){
    enable_moves = -1;
  }
  else if(people == -1 && enable_moves == -1){
    std::cout << "START MOVING" << std::endl;
    enable_moves = 1;
  }
  else if( enable_moves == 1 ){
    struct player pls[people];
    for(int i = 0; i < people; i++){
      ROS_INFO("\t[ %d ] ( %d ) %lf",(int)msg->person[i].id, (int)msg->person[i].dir,(double)msg->person[i].p);
      pls[i].id = (int)msg->person[i].id;
      pls[i].dir = msg->person[i].dir;
      pls[i].dir = pls[i].dir+current_direction;
      pls[i].p = (double)msg->person[i].p;
    }
    p300_send_one(pls,people);
  }
  
  ROS_INFO("DIRECTION : %d\n",this->target_direction);
}

void bot::p300_send_one(player *pls,int n){           // send the p300 command
  hybrid_bci::ParametersConfig config = hybrid_bci::ParametersConfig::__getDefault__();
  double p300_stay_effect = config.p300_stay_effect;
  double stay_stddev = config.stay_stddev;
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
  char* log_msg = (char*)calloc(2000, sizeof(char));
  sprintf(log_msg,"P300 command:\n\tMax probability: %f\n\tSecond probability: %f\n\tDirection offset: %d\n\tCurrent direction: %d",pls[max1].p, pls[max2].p, pls[max1].dir, current_direction);
  std::thread log_th (write_log, log_msg);
  //ROS_INFO("MAX1 %lf MAX2 %lf ", pls[max1].p, pls[max2].p);
  
  mul_distribution(dir.dir, p300_past_effect, max_direction);
  //ROS_INFO("Scaled distribution %lf ",tot_distribution(dir.dir, max_direction));
    
  double ratio = 1 + exp(2*pls[max2].p/pls[max1].p);
  //ROS_INFO("ratio %lf",ratio);
  
  distribution pst;
  pst.dir = (double*)calloc(max_direction,sizeof(double));
  _init_distribution(&pst, (double)max_direction/2, stay_stddev, max_direction);
  mul_distribution(pst.dir, p300_stay_effect, max_direction);
  shift_distribution(pst.dir, current_direction, max_direction);

  distribution k_dist;
  k_dist.dir = (double*)calloc(max_direction,sizeof(double));
  _init_distribution(&k_dist, (double)max_direction/2, p300_stddev*ratio, max_direction); // possibility to add a coefficient whitch depends of the players distance 
  //ROS_INFO("New distribution %lf ",tot_distribution(k_dist.dir, max_direction));
  
  mul_distribution(k_dist.dir, (1-p300_past_effect-p300_stay_effect), max_direction);
  //ROS_INFO("Scaled new distribution %lf ",tot_distribution(k_dist.dir, max_direction));
  
  shift_distribution(k_dist.dir, pls[max1].dir, max_direction);
  sum_distribution(dir.dir, k_dist.dir, max_direction);
  sum_distribution(dir.dir, pst.dir, max_direction);

  double tot = tot_distribution(dir.dir, max_direction);
  mul_distribution(dir.dir, 1/tot, max_direction);
  this->target_direction = calc_direction(dir.dir, max_direction);
  //ROS_INFO("CURRENT DIR : %d ", this->target_direction);
  free(k_dist.dir);
  log_th.join();
}

void bot::motorimageryCallback(const hybrid_bci::motorimagery::ConstPtr& msg)
{
  hybrid_bci::ParametersConfig config = hybrid_bci::ParametersConfig::__getDefault__();
  int max_direction = config.max_direction;
  if(((int)msg->pkg_id) < 0){
    std::cout << "EXIT" << std::endl;
    enable_moves =-2;
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
  char* log_msg = (char*)calloc(1000, sizeof(char));
  sprintf(log_msg, "Motor Imagery Right command\n\tShift: +45\n\tCurrent direction: %d",current_direction);
  std::thread log_th (write_log, log_msg);

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
  log_th.join();
}

void bot::motor_imagery_left(){                                          // send motor imagery left command
  char* log_msg = (char*)calloc(1000, sizeof(char));
  sprintf(log_msg ,"Motor Imagery Left command\n\tShift: -45\n\tCurrent direction: %d",current_direction);
  std::thread log_th (write_log, log_msg);
  
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
  log_th.join();
}

void bot::odomCallback(const nav_msgs::Odometry::ConstPtr &msg){
   
  current_pos = cv::Point2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
  std::cout << current_pos << std::endl;
  double rad = tf::getYaw(msg->pose.pose.orientation);
  double deg = angles::to_degrees(rad);
  if(deg < 0) deg = deg + 360;
  std::cout << "Current direction : " << deg << std::endl;
  current_direction = (int)deg;

  if(target_direction == -1){
    target_direction = current_direction;
    _init_distribution(&dir,180,180, 360);
    shift_distribution( dir.dir, target_direction, 360);
  }
}

int main(int argc, char **argv){
  std::cout<< "Hybrid BCI node starts!!" << std::endl;
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<hybrid_bci::direction_distribution>("move", 10000);
  bot bt;
  auto start_t = std::chrono::system_clock::now();
  std::time_t start_time = std::chrono::system_clock::to_time_t(start_t);
  log_filename = (char*)calloc(100, sizeof(char));
  
  sprintf(log_filename,"%s/Logs/%ld.log", ros::package::getPath("hybrid_bci").c_str(),start_time);
  char* log_msg = (char*)calloc(1000, sizeof(char));
  sprintf(log_msg, "Start!");
  std::thread log_th (write_log, log_msg);
  //std::cout<< log_filename << std::endl;

  bt.current_pos = cv::Point2d(0,0);
  bt.target_direction = -1;
  bt.current_direction = 0;
  bt.dir.dir = (double*)calloc(360,sizeof(double));
  bt.enable_moves = -1;
  

  ros::Subscriber p300_sub = n.subscribe("p300", 1, &bot::p300Callback, &bt);
  ros::Subscriber mi_sub = n.subscribe("motorimagery", 1, &bot::motorimageryCallback, &bt);
  ros::Subscriber od_sub = n.subscribe("odom",1, &bot::odomCallback, &bt);
  ros::Rate loop_rate(1);
  

  int target_dir = bt.target_direction;
  while(ros::ok){
    
    hybrid_bci::direction_distribution msg;
    if(target_dir != bt.target_direction || bt.target_direction == -1){
      
      target_dir = bt.target_direction;
    }
    
    confirm_move(&bt);
    
    
    msg =  bot_move(&bt);
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    if( msg.best_dir == -2){
      exit(0);
    }
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
  
  if(elem.x > 0)res = res;
  if(elem.x < 0)res = -res;
  //if(elem.y > 0)res = -res;
  //if(elem.y < 0)res = res;
  
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
  if(bt->enable_moves == -1){
    return;
  }
  hybrid_bci::ParametersConfig config = hybrid_bci::ParametersConfig::__getDefault__();
  char* log_msg = (char*)calloc(500, sizeof(char));
  sprintf(log_msg, "No-Commands\n\tDirection confirmed");
  std::thread log_th (write_log, log_msg);

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
  log_th.join();

}

hybrid_bci::direction_distribution bot_move( bot *bt){
  ROS_INFO("People %d", bt->enable_moves);
  hybrid_bci::direction_distribution msg;
  std::vector<double> array_msg;
  
  for(int i = 359; i > -1; i--){
    array_msg.push_back(bt->dir.dir[i]);
  }

  if(bt->enable_moves == 1){
    msg.best_dir = bt->target_direction;
  }
  else if(bt->enable_moves == -2){
    msg.best_dir = -2;
  }
  else{
    msg.best_dir = -1;
  }
  msg.directions = array_msg;
  return msg;
}