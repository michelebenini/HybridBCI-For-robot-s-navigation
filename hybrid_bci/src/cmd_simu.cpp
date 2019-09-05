#include "ros/ros.h"
#include "std_msgs/String.h"
#include "stdio.h"
#include <termios.h>

#include <dynamic_reconfigure/server.h>
#include <hybrid_bci/ParametersConfig.h>

#include "hybrid_bci/P300.h"
#include "hybrid_bci/P300_person.h"
#include "hybrid_bci/motorimagery.h"


#include <cv_bridge/cv_bridge.h>

#define MAP_COLS 1000
#define MAP_ROWS 1000

struct player                                             // struct represent player data
{
  char name;
  int dir;
  double p;
  double cost;
};
char getch();
void _init_player(struct player *pl, int n);
void p300_set_fixed(struct player *pls, int n);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_simu");
  ros::NodeHandle n;
  hybrid_bci::ParametersConfig config = hybrid_bci::ParametersConfig::__getDefault__();
  std::cout << "----------COMMANDS!----------"<< std::endl;
  std::cout << "[p] Play"<< std::endl;
  std::cout << "[q] Quit"<< std::endl;
  std::cout << "[a] Motor imagery left of "<< config.mi_degree << " degree" <<std::endl;
  std::cout << "[d] Motor imagery right of "<< config.mi_degree << " degree" <<std::endl;
  std::cout << "[s] Send p300 from [1, 2, 3, 4] each one differs from the previus one of"<< config.max_direction/8 << " degree" <<std::endl;
  std::cout << "[1, 2, 3, 4] Set the p300 target"<< config.max_direction/8 << " degree" <<std::endl;
  std::cout << "-------------END!-----------"<< std::endl;
  
  ros::Publisher mi_pub = n.advertise<hybrid_bci::motorimagery>("motorimagery", 1000);
  ros::Publisher p300_pub = n.advertise<hybrid_bci::P300>("p300", 1000);
  ros::Rate loop_rate(10);
  
  
  int count_mi = 0;
  int count_p300 = 0;
  //ROS_INFO("-- %d -- ",config.max_p300);
  int max_p300 = config.max_p300;
  player pls[max_p300];
  for(int i = 0; i < max_p300; i++){
    _init_player(&pls[i],i);
    //ROS_INFO("PERSON %d - %lf", i ,pls[i].p);
  }

  while (ros::ok())
  {
    int c = getch();
    std::cout << std::endl;
    ROS_INFO("MI[ %d ] P300[ %d ] Command pressed: [ %c ]",count_mi, count_p300,c);
    if( c == 'd'){
      hybrid_bci::motorimagery msg;
      msg.pkg_id = count_mi;
      msg.dir = true;
      mi_pub.publish(msg);
      count_mi++;
    }
    else if( c == 'a'){
      hybrid_bci::motorimagery msg;
      msg.pkg_id = count_mi;
      msg.dir = false;
      mi_pub.publish(msg);
      count_mi++;
    }
    else if( c == 'q'){
      hybrid_bci::motorimagery msg;
      msg.pkg_id = -1;
      msg.dir = false;
      mi_pub.publish(msg);
      count_mi++;
      exit(0);
    }
    else if(c == 's'){
      hybrid_bci::P300 msg;
      msg.pkg_id = count_p300;
      msg.tot_people = max_p300;
      for(int i = 0; i < max_p300; i++){
        hybrid_bci::P300_person p1;
        p1.id = i;
        p1.dir = (45*i)-(config.max_direction/4);
        if(p1.dir >= 0){
          p1.dir = p1.dir + 45;
        }
        p1.p = pls[i].p;
        msg.person.push_back(p1);
      }
      p300_pub.publish(msg);
      count_p300++;
    }
    else if(c == 'p'){
      hybrid_bci::P300 msg;
      msg.pkg_id = count_p300;
      msg.tot_people = -1;
      for(int i = 0; i < 1; i++){
        hybrid_bci::P300_person p1;
        p1.id = -1;
        p1.dir = -1;
        p1.p = -1;
        msg.person.push_back(p1);
      }
      p300_pub.publish(msg);
      count_p300++;
    }else if(c >= '0' && c <= '9' ){

      for(int i = 0; i < max_p300; i++){
        if( pls[i].name+48 == c ){
          p300_set_fixed(pls,i);
        }
        ROS_INFO("PERSON %d - %lf", i ,pls[i].p);
      }
    }
    
    ros::spinOnce();
    loop_rate.sleep();
    
  }
  return 0;
}

char getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings
  char c = (char)getchar();  // read character (non-blocking)
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

void _init_player(struct player *pl, int n){
  hybrid_bci::ParametersConfig config = hybrid_bci::ParametersConfig::__getDefault__();
  int max_p300 = config.max_p300;
  pl->name = (char)n;
  pl->dir = 0;
  pl->p = (double)1/max_p300;
  pl->cost = 0.02;
}
void p300_set_fixed(struct player *pls, int n){                          // set p300 probability in a prefixed way
  hybrid_bci::ParametersConfig config = hybrid_bci::ParametersConfig::__getDefault__();
  double fixed_p300 = config.fixed_p300;
  int max_p300 = config.max_p300; 
  for(int i = 0; i < max_p300; i++){
    if( i == n ){
      pls[i].p = fixed_p300;
    }
    else{
      pls[i].p = (1-fixed_p300)/(max_p300-1);
      
    }
  }
}