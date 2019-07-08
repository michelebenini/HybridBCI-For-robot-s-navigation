#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "iostream"
#include "math.h"
#include "thread"


// drawing parameters
#define MAX_DIMENSION 1000
#define MAP_ROWS 800
#define MAP_COLS 1000
#define DIRECTION_LENGHT 150
#define TARGET_DIRECTION_LENGHT 200

// probability parameters
#define START_MEAN 180
#define START_STD 180
#define FIXED_P300 0.7                                    // probability of selected player in p300
#define P300_STDDEV 5.0                                  // standard deviation of gaussian for p300 commands
#define MI_STDDEV 50.0                                    // standard deviation of gaussian for motor imagery commands
#define STAY_STDDEV 100.0                                 // standard deviation of gaussian to go straight
#define MOVE_STDDEV 10


#define NEW_STAY_EFFECT 0.3                                 // effect of stability on the distribution
#define MOVE_EFFECT 0.03
#define P300_PAST_EFFECT 0.3                              // effect of past commands on the distribution
#define MI_PAST_EFFECT 0.15
#define MI_DEGREE 45

// parameters for Gaus-Marcov Process
#define A_MI 1
#define C_MI 0.2
#define Q_MI 10
#define R_MI 20

#define A_P300 1
#define C_P300 0.8
#define Q_P300 10
#define R_P300 5

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
#define PLAY 'w'                                          // robot rotation to target direction
#define QUIT 'q'                                          // quit command
#define RESTART 'r'
#define MP_SWITCH 'c'
#define MV_SWITCH 'p'
#define P300_G 'o'

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


