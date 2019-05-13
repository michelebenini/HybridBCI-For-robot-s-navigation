#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "iostream"

#define MAX_MOVES  1000                                   // max moves aviable
#define MAX_P300  10                                      // max P300 goals 
#define QUIT 'q'                                          // quit command

struct bot                                                // struct represent the bot data
{
  int radius;                                             // space of moves that bot can do
  int radius_space;                                       // space of rotation for mi command
  int last_cmd;                                           // last mi command received
  int accumulator;                                        // accumulator for the same mi command
  int acc_th;                                             // threshold for accumulator
  cv::Scalar color;
  cv::Point current_pos;          
  cv::Point past_pos[MAX_MOVES];
  int n_moves;
};

struct map                                                // struct represent the current map data 
{
  int rows;
  int cols;
  
  cv::Point start;
  cv::Scalar start_color;
  
  cv::Point goal;
  cv::Scalar goal_color;
};

struct command                                            // struct represent the command received
{
  int time[MAX_P300];
  cv::Point pos[MAX_P300];
  int n_goal;
  int current_goal;
  int mi[MAX_MOVES];
};

struct trajectory
{
  cv::Point pos[MAX_MOVES];
  int dists;
  int middle;
  int dev;
  cv::Point start;
  cv::Point current;
  cv::Point end;
};


void _init_map(struct map *m);                            //  inizialize map
void _init_bot(struct bot *bt);                           //  inizialize bot
void _init_command(struct command *cmd);                  //  inizialize command
void _init_trajectory(struct trajectory *trj);            // inizialize trajectory
void show(struct map m, struct bot bt,struct command cmd);//  show map
void show_command(int i, struct command cmd);             //  show command
void move_bot(struct bot *bt, cv::Point shift);           //  move bot of shift
cv::Point calc_moves(struct map m, struct bot bt);        //  calculate best moves from posizion to goal 
cv::Point perpendicular(cv::Point vec);                   //  calculate perpendicular right vector of vec

int main() {
  map m;
  bot bt;
  command cmd;
  trajectory trj;

  _init_map(&m);
  _init_bot(&bt);
  _init_command(&cmd);
  _init_trajectory(&trj);
  
  // synchronizes the structs
  m.goal = cmd.pos[cmd.current_goal];
  bt.current_pos = m.start;

  std::cout << "Start : " << bt.current_pos << " Goal: " << m.goal << std::endl;

  for(int i = 0; i < MAX_MOVES; i++){
    // calculate best move to goal
    cv::Point shift = calc_moves(m,bt);
    
    // move bot of shift
    move_bot(&bt,shift);

    // show map
    show(m, bt, cmd);
    
    // show command
    show_command(i,cmd);
    
    // if press q quit else speed up the show
    char c = cv::waitKey(1000);
    if(c == QUIT){
      break;
    }

    // calculate distance from point to actual goal
    double dist = sqrt((bt.current_pos.x - m.goal.x)*(bt.current_pos.x - m.goal.x)+(bt.current_pos.y - m.goal.y)*(bt.current_pos.y - m.goal.y)) ;

    // check new command
    if( i > cmd.time[cmd.current_goal]){
      // check P300 command
      if(cmd.current_goal+1 < cmd.n_goal){
        cmd.current_goal++;
        m.goal = cmd.pos[cmd.current_goal];
        std::cout << "Start : " << bt.current_pos << " Goal: " << m.goal << std::endl;
      }
      else{
        break;
      }
    }else{
      // calculate vector perpendicular of left
      cv::Point goal_shift = perpendicular(shift);
      
      // check motor imagery command
      if(cmd.mi[i] == bt.last_cmd){
        if(bt.accumulator < bt.acc_th){
          bt.accumulator++;
        }
      }
      else{
        bt.accumulator = 1;
        bt.last_cmd = cmd.mi[i];
      }

      // shift the goal in base of motor imagery command
      if(cmd.mi[i] == -1){
        m.goal = m.goal + goal_shift*bt.accumulator;
      }else if(cmd.mi[i] == +1){
        m.goal = m.goal - goal_shift*bt.accumulator;
      }
      
      // check distance if recive motor imagery command
      if(dist < bt.radius_space && cmd.mi[i] != 0){
        m.goal = m.goal + shift;
      }
    }   
  }
  
}

void _init_map(struct map *m){
  m->rows = 1000;
  m->cols = 1000;
  m->start = cv::Point(10,10);
  m->start_color = cv::Scalar(0,255,0);
  m->goal = cv::Point(900,400);
  m->goal_color = cv::Scalar(0,0,255);
}

void _init_bot(struct bot *bt){
  bt->radius = 5;
  bt->radius_space = bt->radius*bt->radius;
  bt->color = cv::Scalar(125,125,125);
  bt->current_pos = cv::Point();
  bt->n_moves = 0;
  bt->accumulator = 1;
  bt->last_cmd = 0;
  bt->acc_th = 5;
}

void _init_command(struct command *cmd){
  cmd->n_goal = 3;
  cmd->current_goal = 0;

  cmd->time[0] = 100;
  cmd->pos[0] = cv::Point(900,400);
  
  cmd->time[1] = cmd->time[0] + 100;
  cmd->pos[1] = cv::Point(200,200);

  cmd->time[2] = cmd->time[1] + 100;
  cmd->pos[2] = cv::Point(600,600);

  for(int i = 0; i < MAX_MOVES; i++){
    cmd->mi[i] = 1;
    if(i > 80){
      cmd->mi[i] = 0;
    }
    if(i > 120){
      cmd->mi[i] = -1;
    }
  }
}

void _init_trajectory(struct trajectory *trj){
  trj->start = cv::Point(0,0);
  trj->current = cv::Point(0,0);
  trj->end = cv::Point(0,0);
  trj->dists = 0;
  trj->middle = 0;
  trj->dev = 0;
  for(int i = 0; i < MAX_MOVES; i++)trj->pos[i] = cv::Point(0,0);
}

void show(struct map m, struct bot bt,struct command cmd){
  cv::String window_name = "Map";
  cv::Mat img(m.rows,m.cols, CV_8UC3, cv::Scalar(255,255,255));
  cv::circle(img, m.start, bt.radius_space, m.start_color, 5, 8);
  cv::circle(img, m.goal, bt.radius, m.goal_color, 1, 8);
  cv::circle(img, bt.current_pos, bt.radius, bt.color, -1, 8);
  cv::circle(img, bt.current_pos, bt.radius_space, bt.color, 3, 8);
  for(int i = 0; i < bt.n_moves; i++)cv::circle(img, bt.past_pos[i], bt.radius, bt.color, 1, 8);
  for(int i = 0; i <= cmd.current_goal; i++)cv::circle(img, cmd.pos[i], bt.radius_space, m.goal_color, 5, 8);
  cv::imshow(window_name,img);
  
}

void show_command(int i, struct command cmd){
  cv::Mat img_dir;
  cv::String dir_window = "Command";
  char str_goal[200];
  char str_time[200];
  sprintf(str_goal, "Initial P300 goal : [ %d , %d ]", cmd.pos[cmd.current_goal].x, cmd.pos[cmd.current_goal].y);
  sprintf(str_time, "Command [ %d ]",i);
  
  if(cmd.mi[i]==1){
    img_dir = cv::imread("img/right.jpg", CV_LOAD_IMAGE_COLOR);
  }else if(cmd.mi[i] == -1){
    img_dir = cv::imread("img/left.jpg", CV_LOAD_IMAGE_COLOR);
  }else{
    img_dir = cv::imread("img/up.jpg", CV_LOAD_IMAGE_COLOR);
  }
  
  cv::putText(img_dir, str_goal,cv::Point(10,img_dir.rows-20),cv::FONT_HERSHEY_COMPLEX_SMALL,0.8,cv::Scalar(0,0,0),1,CV_AA);
  cv::putText(img_dir, str_time,cv::Point(90,20),cv::FONT_HERSHEY_COMPLEX_SMALL,0.8,cv::Scalar(0,0,0),1,CV_AA);
  cv::namedWindow( dir_window, cv::WINDOW_NORMAL );
  cv::resizeWindow(dir_window, 400, 400);
  cv::imshow(dir_window,img_dir);

}

void move_bot(struct bot *bt, cv::Point shift){
  bt->past_pos[bt->n_moves] = bt->current_pos;
  bt->current_pos = bt->current_pos + shift;
  bt->n_moves++;
}

cv::Point calc_moves(struct map m, struct bot bt){
  int rdir = bt.radius;
  int dir = ((rdir*2)+1)*((rdir*2)+1);
  cv::Point moves[dir];
  double dists[dir];
  int k = 0;
  for(int i = -rdir; i <= rdir; i++){
    for(int j = -rdir; j <= rdir; j++){
      moves[k].x = bt.current_pos.x + i;
      moves[k].y = bt.current_pos.y + j;
      
      double dx = moves[k].x - m.goal.x;
      double dy = moves[k].y - m.goal.y;
      dx = dx*dx; 
      dy = dy*dy;
      double angle = sqrt((i*i)+(j*j));
      if(i == 0 && j == 0){
        angle = 1;
      }
      dists[k] = sqrt(dx+dy) + angle;
      k = k + 1;
      //std::cout << "\t[ "<<k<<" ] "<<dists[k]<<std::endl;
    }
  }

  int index = 0;
  for( int i = 0; i < k; i++){
    //std::cout << "[ " << i << " ] " << dists[i] << "\t" << moves[i] << std::endl;
    if(dists[index] > dists[i]){
      index = i;
    }
  }
  cv::Point pos = moves[index]-bt.current_pos;
  return pos;
}

cv::Point perpendicular(cv::Point vec){
    cv ::Point shift(0,0);
    if(vec.x > 0 && vec.y > 0){
        shift.y = -vec.x;
        shift.x = vec.y;
    }
    else if(vec.x < 0  && vec.y > 0){
        shift.y = -vec.x;
        shift.x = vec.y;
    }
    else if(vec.x > 0  && vec.y < 0){
         shift.y = -vec.x;
         shift.x = vec.y;
    }
    else if(vec.x < 0  && vec.y < 0){
         shift.y = -vec.x;
         shift.x = vec.y;
    }
    else if(vec.x == 0){
         shift.y = 0;
         shift.x = vec.y;
    }
    else if(vec.y == 0){
         shift.y = -vec.x;
         shift.x = 0;
    }
    return shift;
}

/*
  - Il goal dato dalla è certo?
  - Mi devono spostare il goal o cambiano solo la traiettoria?
*/