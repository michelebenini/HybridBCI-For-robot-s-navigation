#include "parameters.h"

void _init_player(struct player *pl, int n);
void _init_bot(struct bot *bt);
void _init_distribution(struct distribution *dis, double mean, double std_dev);
double normal_distribution_calc(int n, double mean, double std_dev);

void show_map(struct bot bt, struct player pls[MAX_P300]);
void show_distribution(struct distribution dis);

void p300_set(struct player pls[MAX_P300], int n);
void p300_set_fixed(struct player pls[MAX_P300], int n);
void motor_imagery_right(struct bot *bt);
void motor_imagery_left(struct bot *bt);
void p300_send(struct distribution *dis, player pls[MAX_P300], bot bt);
void p300_check(struct player pls[MAX_P300], struct bot bt);

void sum_distribution(double *out, double *in);
void shift_distribution(double *in, int shift);
void mul_distribution(double *out, double mul);
double tot_distribution(double *in);

int calculate_degree(cv::Point bt, cv::Point elem);
double dist(cv::Point p1, cv::Point p2);
int calc_direction(double *in);
cv::Point calculate_point(cv::Point bt, int dg, double dist);

int main() {
  player pls[MAX_P300];
  for(int i = 0; i < MAX_P300; i++){_init_player(&pls[i],i);}

  bot bt;
  _init_bot(&bt);
     
  char cmd;
  int p300_switch = 0;
  int dis = 0;
  
  std::cout << "\t\tHELP\t " << std::endl;
  std::cout << "[ 0 - MAX_P300 ] set p300 probability" << std::endl;
  std::cout << "[ s ] send p300 " << std::endl;
  std::cout << "[ d ] send motor imagery right command " << std::endl;
  std::cout << "[ a ] send motor imagery left command " << std::endl;
  std::cout << "[ m ] switch p300 setter mode" << std::endl;
  std::cout << "[ p ] robot moves " << std::endl;
  std::cout << "[ h ] help " << std::endl;
  
  while(true){
    show_map( bt, pls);
    cmd = cv::waitKey(0);
    std::cout << std::endl << "Command received : [ " << cmd << " ]" << std::endl;

    if(cmd == QUIT){
      break;
    }
    else if(cmd >= '0' && cmd <= '9' ){
      for(int i = 0; i < MAX_P300; i++){
        if( pls[i].name+48 == cmd ){
          if(p300_switch == 0){
            p300_set_fixed(pls,i);
            p300_check(pls,bt);
          }else{
            p300_set(pls,i);
            p300_check(pls,bt);
          }
        }
      }
    }else if(cmd == R){
      motor_imagery_right(&bt);
    }
    else if(cmd == L){
      motor_imagery_left(&bt);
    }
    else if(cmd == P300){
      p300_send(&bt.dir, pls, bt);
    }
    else if(cmd == P300_switch){
      if(p300_switch == 0)
        p300_switch=1;
      else 
        p300_switch = 0;
    }
    else if( cmd == PLAY){  
      dis = bt.current_direction - bt.target_direction;
      if((dis >= - MAX_DIRECTION/2) && (dis < 0 || dis >= MAX_DIRECTION/2)){
        bt.current_direction++;
        bt.current_direction = bt.current_direction%360;
      }
      else if((dis <= MAX_DIRECTION/2) && (dis > 0 || dis <= -MAX_DIRECTION/2)){
        bt.current_direction--;
        if(bt.current_direction < 0){
          bt.current_direction = 360 - bt.current_direction;
        }
      }
      p300_check(pls,bt);
    }
    else{
      std::cout << "\t\tHELP\t " << std::endl;
      std::cout << "[ 0 - MAX_P300 ] set p300 probability" << std::endl;
      std::cout << "[ s ] send p300 " << std::endl;
      std::cout << "[ d ] send motor imagery right command " << std::endl;
      std::cout << "[ a ] send motor imagery left command " << std::endl;
      std::cout << "[ m ] switch p300 setter mode" << std::endl;
      std::cout << "[ p ] robot moves " << std::endl;
      std::cout << "[ h ] help " << std::endl;

    }
    bt.target_direction = calc_direction(bt.dir.dir);
    dis = bt.current_direction - bt.target_direction;
    std::cout << "Current direction : " << bt.current_direction << std::endl;
    std::cout << "Target direction : " << bt.target_direction << std::endl;
    std::cout << "Distance : " << abs(dis) << "°" << std::endl;
  }

}

void _init_player(struct player *pl, int n){
  int x = (int)((n+1)*MAP_COLS/(MAX_P300+1));
  int y = (int)(0.1 * MAP_ROWS);
  pl->name = (char)n;
  pl->pos = cv::Point(x,y);
  pl->p = (double)1/MAX_P300;
  pl->size = 50;
  pl->color =  cv::Scalar(0,0,255);
  pl->cost = 0.02;
}

void _init_bot(struct bot *bt){
  int x = (int)MAP_COLS/2;
  int y = (int)(MAP_ROWS - (0.3 * MAP_ROWS));
  bt->current_pos = cv::Point(x,y);
  bt->radius = 5;
  bt->radius_area = 60;
  bt->radius_dist = 120;
  bt->color =  cv::Scalar(0,0,0);
  _init_distribution(&bt->dir,(double)MAX_DIRECTION/2, 90.0 );
  bt->current_direction = MAX_DIRECTION/2;
  bt->target_direction = MAX_DIRECTION/2;
}

void _init_distribution(struct distribution *dis, double mean, double std_dev){
  dis->mean = mean;
  dis->std_dev = std_dev;
  for(int i = 0; i < MAX_DIRECTION; i++){
    dis->dir[i] = normal_distribution_calc(i, mean, std_dev); 
  }
}

double normal_distribution_calc(int n, double mean, double std_dev){
  double res = (1/sqrt(2*M_PI*std_dev*std_dev))*exp(-(n-mean)*(n-mean)/(2*std_dev*std_dev));
  return res;
}

void show_map( struct bot bt, struct player pls[MAX_P300]){
  cv::Mat map(cv::Size(MAP_COLS, MAP_ROWS), CV_8UC3, cv::Scalar(255,255,255));
  cv::String window_name = "Map";
  cv::Mat img = map.clone();

  // draws FOV
  cv::Point pt3 = calculate_point(bt.current_pos, bt.current_direction-FOV,LENGHT_FOW );
  cv::line(img,bt.current_pos,pt3,cv::Scalar(0,0,0),2,CV_AA,0);

  cv::Point pt4 = calculate_point(bt.current_pos, bt.current_direction+FOV,LENGHT_FOW );
  cv::line(img,bt.current_pos,pt4,cv::Scalar(0,0,0),2,CV_AA,0);

  cv::Point rook_points[1][3] = { bt.current_pos, pt3, pt4};
  const cv::Point* ppt[1] = { rook_points[0] };
  int npt[] = {3};
  cv::fillPoly(img,ppt, npt,1,cv::Scalar(200,200,200),CV_AA, 0,cv::Point());

  cv::circle(img, bt.current_pos, bt.radius, bt.color, -1, CV_AA);
  cv::circle(img, bt.current_pos, bt.radius_area, bt.color, -1, CV_AA);
  cv::circle(img, bt.current_pos, bt.radius_dist, bt.color, 1, CV_AA);
  
  cv::Vec3b color;color[0] = 0;color[1] = 0;color[2] = 0;
  
  double max = -1;
  double coef_r = 0;
  double coef_c = 0;
  for(int i = 0; i < MAX_DIRECTION; i++)
    if(max < bt.dir.dir[i])
      max = bt.dir.dir[i];
  coef_r = (bt.radius_dist-bt.radius_area)/max;  
  coef_c = 255/max; 
    
  for(int i = 0; i < MAX_DIRECTION; i++){
    color[1] = (int)(coef_c * bt.dir.dir[i]);
    for(int j = bt.radius_area; j < bt.radius_dist && j < bt.radius_area+(coef_r*bt.dir.dir[i]); j++){
      cv::ellipse(img, bt.current_pos, cv::Size(j,j), 0.0, (double)i+(MAX_DIRECTION/4) , (double)(i + 1)+(MAX_DIRECTION/4),color, 2, CV_AA,0 );
    }
    
  }

  
  cv::Point pt1 = calculate_point(bt.current_pos, bt.target_direction, TARGET_DIRECTION_LENGHT);
  cv::arrowedLine(img, bt.current_pos, pt1, cv::Scalar(100,100,100), 2, CV_AA,0);
  
  cv::Point pt2 = calculate_point(bt.current_pos, bt.current_direction, DIRECTION_LENGHT);
  cv::arrowedLine(img, bt.current_pos, pt2, cv::Scalar(0,0,0), 2, CV_AA,0);

  // draws players
  for(int i = 0; i < MAX_P300; i++){
    char str[200];
    sprintf(str, "[ %d ] %0.2lf",(int)pls[i].name,pls[i].p);
    cv::Point pos = pls[i].pos;
    pos.x = pos.x-(pls[i].size/2);
    cv::circle(img, pls[i].pos, pls[i].size, pls[i].color, (int)(pls[i].p*20), CV_AA);
    cv::putText(img, str,pos,cv::FONT_HERSHEY_COMPLEX_SMALL,0.5,cv::Scalar(0,0,0),1,CV_AA);
    }
 
  cv::imshow(window_name,img);
}

void show_distribution(struct distribution dis){
  char window_name[200];
  sprintf(window_name, "Distribution");
  int r = 300;
  cv::Mat img(cv::Size(MAX_DIRECTION, r), CV_8UC3, cv::Scalar(255,255,255));
  cv::Vec3b color;
  color[0] = 0;
  color[1] = 0;
  color[2] = 0;

  double max = -1;
  for(int i = 0; i < MAX_DIRECTION; i++){
      if( dis.dir[i] > max)max= dis.dir[i];
  }

  for(int i = 0; i < MAX_DIRECTION; i++){
    for(int j = 0; j < dis.dir[i]*(r*2/3)/max; j++){
      img.at<cv::Vec3b>(cv::Point(i,img.rows-j)) = color;
    }
  }
  cv::imshow(window_name,img);
}

void p300_set(struct player pls[MAX_P300], int n){                                // set p300 probability 
  for(int i = 0; i < MAX_P300; i++){
    if( i == n ){
      pls[i].p = pls[i].p + pls[i].cost * ((double)(MAX_P300-1));
    }
    else{
      pls[i].p = pls[i].p - pls[i].cost;
      if(pls[i].p < 0){
        pls[n].p = pls[n].p + pls[i].p;
        pls[i].p = 0;
      }
    }
  }
}

void p300_set_fixed(struct player pls[MAX_P300], int n){                          // set p300 probability in a prefixed way
  for(int i = 0; i < MAX_P300; i++){
    if( i == n ){
      pls[i].p = FIXED_P300;
    }
    else{
      pls[i].p = (1-FIXED_P300)/(MAX_P300-1);
      
    }
  }
}

void p300_check(struct player pls[MAX_P300], struct bot bt){                       // FOV-based control of p300 probability 
  for(int i = 0; i < MAX_P300; i++){
    int dg_p = calculate_degree(bt.current_pos, pls[i].pos)-MAX_DIRECTION/2;
    if(dg_p > MAX_DIRECTION)dg_p = dg_p%MAX_DIRECTION;
    if(dg_p < 0 ) dg_p = MAX_DIRECTION + dg_p;
    int dg_l = bt.current_direction - FOV;
    int dg_r = bt.current_direction + FOV;
    if(dg_l > dg_p || dg_r < dg_p){
      pls[i].p = 0;
    }
    else if(dg_l < dg_p && dg_r > dg_p && (pls[i].p == 0 || pls[i].p != pls[i].p)){
      pls[i].p = (double)1/MAX_P300;
    }
  }
  double sum = 0;
  for(int i = 0; i < MAX_P300; i++){
    sum = sum + pls[i].p;
  }
  for(int i = 0; i < MAX_P300; i++){
    pls[i].p = pls[i].p / sum;
  }
}

void motor_imagery_right(struct bot *bt){                                          // send motor imagery right command
  std::cout << "Right command!" << std::endl;

  distribution mi;
  _init_distribution(&mi, (double)MAX_DIRECTION/2, MI_STDDEV);
  mul_distribution(mi.dir, NEW_MI_EFFECT);
  int shift = (bt->current_direction-(MAX_DIRECTION/4));
  shift_distribution(mi.dir, shift);
  
  distribution k_dist;
  _init_distribution(&k_dist, (double)MAX_DIRECTION/2, STAY_STDDEV);
  mul_distribution(k_dist.dir, NEW_STAY_EFFECT);
  
  mul_distribution(bt->dir.dir, PAST_EFFECT);
  sum_distribution(bt->dir.dir, mi.dir);

  shift_distribution( k_dist.dir, (bt->current_direction-MAX_DIRECTION/2));
  sum_distribution(bt->dir.dir, k_dist.dir);
  
  double tot = tot_distribution(bt->dir.dir);
  mul_distribution(bt->dir.dir, 1/tot);
}

void motor_imagery_left(struct bot *bt){                                          // send motor imagery left command
  std::cout << "Left command!" << std::endl;
  
  distribution mi;
  _init_distribution(&mi, (double)MAX_DIRECTION/2, MI_STDDEV);
  mul_distribution(mi.dir, NEW_MI_EFFECT);
  int shift = (bt->current_direction+(MAX_DIRECTION/4));
  shift_distribution(mi.dir, shift);
  
  distribution k_dist;
  _init_distribution(&k_dist, (double)MAX_DIRECTION/2, STAY_STDDEV);
  mul_distribution(k_dist.dir, NEW_STAY_EFFECT);
  
  mul_distribution(bt->dir.dir, PAST_EFFECT);
  sum_distribution(bt->dir.dir, mi.dir);

  shift_distribution( k_dist.dir, (bt->current_direction-MAX_DIRECTION/2));
  sum_distribution(bt->dir.dir, k_dist.dir);
  
  double tot = tot_distribution(bt->dir.dir);
  mul_distribution(bt->dir.dir, 1/tot);
}

void p300_send(struct distribution *dis, player pls[MAX_P300], bot bt){           // send the p300 command
  std::cout << "P300 command!" << std::endl;
  for(int i = 0; i < MAX_P300; i++){
    distribution k_dist;
    _init_distribution(&k_dist, (double)MAX_DIRECTION/2, P300_STDDEV); // possibility to add a coefficient whitch depends of the players distance 
    mul_distribution(k_dist.dir, pls[i].p*NEW_P300_EFFECT);
    int shift = calculate_degree(bt.current_pos, pls[i].pos);
    shift_distribution(k_dist.dir, shift);
    mul_distribution(dis->dir, PAST_EFFECT);
    sum_distribution(dis->dir, k_dist.dir);
  }
  double tot = tot_distribution(dis->dir);
  mul_distribution(dis->dir, 1/tot);
}

void sum_distribution(double *out, double *in){
  for(int i = 0; i < MAX_DIRECTION; i++){
    out[i] = out[i] + in[i];
  }
}

void shift_distribution(double *out, int shift){
  double in[MAX_DIRECTION];
  for(int i = 0; i < MAX_DIRECTION; i++){
    in[i] = out[i];
  }

  for(int i = 0; i < MAX_DIRECTION; i++){
    int index = (shift+i);
    if(index > MAX_DIRECTION)index = index%MAX_DIRECTION;
    if(index < 0)index = MAX_DIRECTION+index;
    out[index] = in[i];
  }
  
}

void mul_distribution(double *out, double mul){
  if(mul < 0)return;
  for(int i = 0; i < MAX_DIRECTION; i++){
    out[i] = out[i] * mul;
  }
}

double tot_distribution(double *in){
  double res = 0;
  for(int i = 0; i < MAX_DIRECTION; i++){
    res = res + in[i];
  }
  return res;
}

int calculate_degree(cv::Point bt, cv::Point elem){                               // calculate degree beetwen two points
  double d = dist(bt, elem);
  cv::Point pt0(0,d);
  elem = elem - bt;

  double dp = elem.x*pt0.x + elem.y*pt0.y ;
  double norm = d*sqrt(elem.x*elem.x + elem.y*elem.y);
  double cosin = dp / norm;
  double res = acos(cosin) * (180.0 / M_PI);
  
  if(elem.x > 0)res = 360-(res+180);
  if(elem.x < 0)res = res+180;
  
  res = res*MAX_DIRECTION/360;
  int result = (((int)res)%360);
  return result;
}

double dist(cv::Point p1, cv::Point p2){                                          // calculate the Euclidean distance beetwen two points
  return sqrt((p1.x - p2.x)*(p1.x - p2.x)+(p1.y - p2.y)*(p1.y - p2.y)) ;
}

int calc_direction(double *in){                                                   // calculate the target direction basing on distribution
  double acc[MAX_DIRECTION];
  for(int i = 0; i < MAX_DIRECTION; i++){
    acc[i] = 0;
    for(int j = -WINDOW_DIRECTION; j < WINDOW_DIRECTION; j++){
      int index = i + j;
      if(index < 0){
        index = MAX_DIRECTION + index;
      }
      else if(index >= MAX_DIRECTION){
        index = index - MAX_DIRECTION;
      }
      acc[i] = acc[i] + in[index];
    }
  }
  int max_pos = 0;
  for(int i = 1; i < MAX_DIRECTION; i++){
      if(acc[i] > acc[max_pos]){
        max_pos = i;
      }
  }
  return max_pos;
}


cv::Point calculate_point(cv::Point bt, int dg, double dist){                     // calculare the point at distance dist and degree dg from bt
  cv::Point pt;

  double angle = (double)dg*M_PI/180;
  double s = dist*sin(angle);
  double c = dist*cos(angle);

  pt.x = bt.x - (int)s;
  pt.y = bt.y + (int)c;

  return pt;
}