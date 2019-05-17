#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "chrono"
#include "random"
#include "iostream"
#include "math.h"

#define MAP_ROWS 800
#define MAP_COLS 1000
#define DIRECTION_LENGHT 150
#define TARGET_DIRECTION_LENGHT 200

#define MAX_MOVES  1000                                   // max moves aviable
#define MAX_P300  4                                       // max P300 goals 
#define FIXED_P300 0.7
#define P300_STDDEV 10.0
#define MI_STDDEV 90.0
#define STAY_STDDEV 360.0
#define QUIT 'q'                                          // quit command

#define MAX_DIRECTION 360                                 // possible direction 
#define WINDOW_DIRECTION 1
#define FOV 45
#define LENGHT_FOW 1000

#define R 'd'                                             // right motor-imagery command
#define L 'a'                                             // left motor-imagery command
#define P300 's'                                          // send p300 command
#define P300_switch 'm'                                   // switch to p300 manual setting
#define PLAY 'p'

struct distribution
{
  double dir[MAX_DIRECTION];
  double mean;
  double std_dev;
};

struct player                                             // struct represent player data
{
  char name;
  cv::Point pos;
  double p;
  int size;
  cv::Scalar color;
  double cost;
};

struct bot                                                // struct represent the bot data
{
  cv::Point current_pos;
  int radius;
  int radius_area;
  int radius_dist;
  int current_direction;
  int target_direction;
  cv::Scalar color;
  distribution dir;
};


