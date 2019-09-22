#include "ros/ros.h"
#include "std_msgs/String.h"
#include "stdio.h"
#include <termios.h>

#include <dynamic_reconfigure/server.h>
#include <hybrid_bci/ParametersConfig.h>

#include "hybrid_bci/P300.h"
#include "hybrid_bci/P300_person.h"
#include "hybrid_bci/motorimagery.h"

#include "cnbiros_tobi_msgs/TidMessage.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>


#define ROWS 200
#define COLS 100

char getch();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sendsp300");
    ros::NodeHandle n;
    hybrid_bci::ParametersConfig config = hybrid_bci::ParametersConfig::__getDefault__();
    std::cout << "----------COMMANDS!----------"<< std::endl;
    std::cout << "[1] Sends P300 on p1.jpg"<< std::endl;
    std::cout << "[2] Sends P300 on p2.jpg"<< std::endl;
    std::cout << "[3] Sends P300 on p3.jpg"<< std::endl;
    std::cout << "[4] Sends P300 on p4.jpg"<< std::endl;
    std::cout << "-------------END!-----------"<< std::endl;

    ros::Publisher pub =  n.advertise<cnbiros_tobi_msgs::TidMessage>("/rostid_cnbi2ros", 10000);
    cv::Mat img;
    int target_code = 34817;
    ros::Rate loop_rate(10);
    char* name;
    name = (char*)calloc(1000, sizeof(char));
    sprintf(name,"Target selected");
    int count = 0;
    int c;
    
    while (ros::ok())
    {
        c = getch();
        int index = -1;
        
        std::cout << std::endl;
        std::vector<std::string> filename = {"p1.jpg", "p2.jpg", "p3.jpg", "p4.jpg"};
        if(c=='q'){
            exit(0);
        }
        else if(c == '1'){
            index = 0;
        }
        else if(c == '2'){
            index = 1;
        }
        else if(c == '3'){
            index = 2;
        }
        else if(c == '4'){
            index = 3;
        }
        
              

        if(index == -1){
            continue;
        }
        else{
            cnbiros_tobi_msgs::TidMessage msg;

            msg.description = filename[index].c_str();
            msg.pipe = config.p300_pipe;
            msg.event = target_code + index;
            msg.family = config.family;
            msg.header.frame_id = count;
            msg.header.seq = count;
            msg.header.stamp = ros::Time::now();
            msg.version = config.version;
            count++;

            pub.publish(msg);
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

