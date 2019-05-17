#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "iostream"
#include "math.h"

// drawing parameters
#define MAP_ROWS 800
#define MAP_COLS 1000
#define DIRECTION_LENGHT 150
#define TARGET_DIRECTION_LENGHT 200

// probability parameters
#define FIXED_P300 0.7                                    // probability of selected player in p300
#define P300_STDDEV 20.0                                  // standard deviation of gaussian for p300 commands
#define MI_STDDEV 100.0                                    // standard deviation of gaussian for motor imagery commands
#define STAY_STDDEV 180.0                                 // standard deviation of gaussian to go straight

#define NEW_P300_EFFECT 1                                 // effect of p300 on the distribution
#define NEW_MI_EFFECT 1                                   // effect of Motor imagery on the distribution
#define NEW_STAY_EFFECT 1                                 // effect of stability on the distribution
#define PAST_EFFECT 2                                     // effect of past commands on the distribution

// bot parameters
#define MAX_DIRECTION 360                                 // possible angles 
#define WINDOW_DIRECTION 1                                
#define FOV 45                                            // field of view
#define LENGHT_FOW 1000                                   // lenght of field of view

// commands
#define MAX_P300  4                                       // max P300 goals 
#define R 'd'                                             // right motor-imagery command
#define L 'a'                                             // left motor-imagery command
#define P300 's'                                          // send p300 command
#define P300_switch 'm'                                   // switch to p300 manual setting
#define PLAY 'p'                                          // robot rotation to target direction
#define QUIT 'q'                                          // quit command

struct distribution                                       // struct represent normal distribution
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
  int radius_dist;                                        // radius to draw the distribution probability of directions
  int current_direction;
  int target_direction;
  cv::Scalar color;
  distribution dir;
};


