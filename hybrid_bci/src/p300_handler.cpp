#include "ros/ros.h"
#include <ros/package.h>
#include "iostream"
#include <thread>         // std::thread
#include <mutex>          // std::mutex

#include <image_transport/image_transport.h>
#include <face_classification/ClassifyAll.h>
#include <face_classification/RegisterFace.h>
#include <ros/service.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "hybrid_bci/P300.h"
#include "hybrid_bci/P300_person.h"

#include "cnbiros_tobi_msgs/TidMessage.h"
#include <hybrid_bci/ParametersConfig.h>

struct person_roi
{
  int id;
  cv::Point corner;
  int height;
  int width;
  int active; 
};

class view{
  public:
    double mi_left;
    double mi_right;
    int count;
    person_roi *pls;
    int n_person;
    cv::Mat img;
    std::vector<std::string> filename;
    int active;
    ros::Publisher pub2cnb;
    ros::Publisher pubcmd;
        
    view(ros::ServiceClient c);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void cnbiCallback(const cnbiros_tobi_msgs::TidMessage::ConstPtr& msg);
    void run_face_detector(ros::ServiceClient c, int counter);
    void activation();
    void send_attemp_off();
};

std::mutex mtx_img;
std::mutex mtx_pls;


void view::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  char str[200];
  try
  {
    ///////////////////////////////////////////////////////////// TEST    ///////////////////////////////////////////
    cv::Mat current;
    while(!mtx_img.try_lock()){}
      //img = cv::imread("/home/michele/catkin_ws/src/hybrid_bci/data/all.png");
      img = cv_bridge::toCvShare(msg, "bgr8")->image;
      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      current = img.clone();
      mtx_img.unlock();
    
    if(current.empty()){
      ROS_INFO("Cam not recording!");
      return;
    }
    else{
      ROS_INFO("Cam is working!");
    }
    ROS_INFO("People found : %d", n_person);
    while(!mtx_pls.try_lock()){};
      for(int i = 0; i < n_person; i++){
        if(pls[i].id == -1)continue;
        //ROS_INFO("[ %d ] Person %s!", i, filename[pls[i].id].c_str());
        cv::Rect rect = cv::Rect(pls[i].corner.x,pls[i].corner.y,pls[i].width,pls[i].height);
        
        if(pls[i].active == 1){
          cv::Mat roi = current(rect);
          roi.setTo(cv::Scalar(0, 255, 0));
        }
        else{
          cv::rectangle(current, rect, cv::Scalar(0, 0, 0),2);
        }
      }
      mtx_pls.unlock();
   
    cv::imshow("CamP300", current);
    char q = cv::waitKey(1);
    if(q =='q'){
      cv::destroyWindow("CamP300");
      exit(0);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void view::cnbiCallback(const cnbiros_tobi_msgs::TidMessage::ConstPtr& msg){


  hybrid_bci::ParametersConfig config = hybrid_bci::ParametersConfig::__getDefault__();
  int event = msg->event;
  std::cout<< std::endl << std::endl << "VERIFICARE" << std::endl;

  if(event >= config.p300_task && event < config.p300_task+4){
    int prs = event - config.p300_task;
    int pos_deg = 0;
    //std::cout << "Person selected : " << prs << std::endl;
    //std::cout << "Corner : (x " << pls[prs].corner.x << ", y "<< pls[prs].corner.y <<" )" << std::endl;
    //std::cout << "Width : " << pls[prs].width << " Height :  "<<pls[prs].height << std::endl;
    
    while(!mtx_pls.try_lock()){}
      pos_deg = pls[prs].corner.x + (pls[prs].width/2);
      mtx_pls.unlock();
    double ratio = 0;  
    while(!mtx_img.try_lock()){}
      ratio = (double)pos_deg/img.cols;
      mtx_img.unlock();
    
    pos_deg = (ratio*config.kinet_fov)-(config.kinet_fov/2);
    std::cout<< "Degree of person selected : " <<pos_deg<<std::endl;
    hybrid_bci::P300 msg_p;
    msg_p.pkg_id = count;
    msg_p.tot_people = config.max_p300;
    for(int i = 0; i < msg_p.tot_people ;i++){
      hybrid_bci::P300_person p1;
      p1.id = i;
             
      if(i == prs){
        p1.dir = pos_deg;
        p1.p = config.fixed_p300;
      }
      else{
        p1.dir = 0;
        p1.p = (1-config.fixed_p300)/(config.max_p300-1);
      }
      msg_p.person.push_back(p1);
    }
    pubcmd.publish(msg_p);
  }
}

view::view(ros::ServiceClient c){
  // setup dati ritornati dal classificatore
  // come nome faccie associate ad id
  active = -1;
  //std::vector<std::string> fn = {"emg.jpg", "pagello.jpg", "ghidoni.jpg", "michele.jpg", "daniele.jpg", "miche.jpg", "giorgio.jpg", "kenji.jpg"};
  std::vector<std::string> fn = {"emg.jpg", "miche.jpg", "giorgio.jpg", "kenji.jpg"};
  filename = fn;
  pls = (person_roi*)calloc(filename.size(), sizeof(person_roi));
  std::string path = ros::package::getPath("hybrid_bci");
  for(int i = 0; i < filename.size(); i++){
    
    char f[200];
    sprintf(f , "%s/data/%s", path.c_str(), filename[i].c_str());
    std::cout << "Loading file : " << f << std::endl;

    cv::Mat face = cv::imread(f);
    if(face.empty()){
      ROS_INFO("Face is empty!");
      continue;
    }
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg; // >> message to be sent
    face_classification::RegisterFace req;
    
    std_msgs::Header header; // empty header
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, face);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
    req.request.image = img_msg;
		req.request.name.data = filename[i];
		if(!c.call(req))
    {
      ROS_INFO("Register faces FAILED!");
      exit(0);
    }
  }
}
void view::run_face_detector(ros::ServiceClient c, int counter){
  // esegue il servizio
  // immagine su cui esegue <img>
  face_classification::ClassifyAll req;
  while(!mtx_img.try_lock()){}
    cv::Mat cam = img.clone(); // << image MUST be contained here
    mtx_img.unlock();
  
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg; // >> message to be sent

  std_msgs::Header header; // empty header
  header.seq = counter; // user defined counter
  header.stamp = ros::Time::now(); // time
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, cam);
  img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
  req.request.image = img_msg;

  ROS_INFO("Sending end attemp!");
  send_attemp_off();
  ROS_INFO("Sending Request!");
  if(!c.call(req))
  {
    ROS_INFO("Classification FAILED!");
    return;
  }
  ROS_INFO("Counting people known!");
  
  
  while(!mtx_pls.try_lock()){};
    int done[filename.size()];
    for(int i = 0; i < filename.size(); i++){
      pls[i].corner = cv::Point(0,0);
      pls[i].height = 0;
      pls[i].width = 0;
      pls[i].id = -1;
      pls[i].active = -1;
      done[i] = 0;
    }
    n_person = 0;
    ///////////////////////////////////////////////////////////////////////////////////////////
    for(int i = 0; i < req.response.results.size() ; i++){
      for(int j = 0; j< filename.size(); j++){
        if(filename[j].compare(req.response.results[i].person_name)==0 && done[j] != 1)
        {
          pls[n_person].corner = cv::Point(req.response.results[i].face_region.center.x - req.response.results[i].face_region.size_x / 2, req.response.results[i].face_region.center.y - req.response.results[i].face_region.size_y / 2);
          pls[n_person].height = req.response.results[i].face_region.size_y;
          pls[n_person].width = req.response.results[i].face_region.size_x;
          pls[n_person].id = j;
          pls[n_person].active = -1;
          done[j] = 1;
          n_person++;
          ROS_INFO("[ j %d, i %d ] Person found %s!", j, i,req.response.results[i].person_name.c_str());
          std::cout << "Corner : " << pls[i].corner << std::endl;
          std::cout << "Height : " << pls[i].height << std::endl;
          std::cout << "Width : " << pls[i].width << std::endl;
        }
      }
    
    
    
    mtx_pls.unlock();
  }
  
  ROS_INFO("Detection done!");
}

void view::activation(){
  ROS_INFO("Activation between %d people!",n_person);
  hybrid_bci::ParametersConfig config = hybrid_bci::ParametersConfig::__getDefault__();
  while(!mtx_pls.try_lock()){};
  
  if(n_person < 1){
    active = -1;
    mtx_pls.unlock();
    return;
  }
  if(active != -1){
    pls[active].active = -1;
  }
  int r = rand(); //random
  active = (r%(n_person));
  ROS_INFO("Activate : %d", active);
  cnbiros_tobi_msgs::TidMessage msg;

  msg.description = filename[pls[active].id].c_str();
  msg.pipe = config.p300_pipe;
  msg.event = (int)config.p300_task + pls[active].id;
  msg.event = (int)msg.event + (int)config.flash;
  msg.family = config.family;
  msg.header.frame_id = count;
  msg.header.seq = count;
  msg.header.stamp = ros::Time::now();
  msg.version = config.version;

  

  pls[active].active = 1;
  pub2cnb.publish(msg);
  ROS_INFO("Person activated is ( %d ) %s",active, filename[pls[active].id].c_str());

  count++;
  mtx_pls.unlock();
}

void view::send_attemp_off(){
  cnbiros_tobi_msgs::TidMessage msg;
  hybrid_bci::ParametersConfig config = hybrid_bci::ParametersConfig::__getDefault__();
  msg.description = "Attemp off";
  msg.pipe = config.p300_pipe;
  msg.event = (int)config.attemp + (int)config.off;
  msg.family = config.family;
  msg.header.frame_id = "";
  msg.header.seq = count;
  msg.header.stamp = ros::Time::now();
  msg.version = config.version;

  pub2cnb.publish(msg);
}

int main(int argc, char **argv){
    // controllare weight and hight sul run
    ros::init(argc, argv, "CamP300");
    ros::NodeHandle n;
    ROS_INFO("Node start!");

    ros::ServiceClient srv_r = n.serviceClient<face_classification::RegisterFace>("/register_face",true);
    ros::ServiceClient srv_ca = n.serviceClient<face_classification::ClassifyAll>("/classify_all", true);

    view v = view(srv_r);
    v.pub2cnb = n.advertise<cnbiros_tobi_msgs::TidMessage>("rostid_ros2cnbi", 10000);
    v.pubcmd = n.advertise<hybrid_bci::P300>("p300", 10000);
    
    v.count = 0;
    v.n_person = -1;

    cv::namedWindow("CamP300", CV_WINDOW_NORMAL);
    image_transport::ImageTransport it(n);
    image_transport::Subscriber img_sub = it.subscribe("camera/rgb/image_raw", 1, &view::imageCallback, &v);
    ros::Subscriber p300_sub = n.subscribe("rostid_cnbi2ros", 1, &view::cnbiCallback, &v);

    ros::Rate loop_rate(2);
    
    for(int counter = 0; ros::ok; counter++){
      if(!v.img.empty()){
        if(counter%6 == 0){
          ROS_INFO("RUN face detector");
          v.run_face_detector(srv_ca, counter);
          ROS_INFO("END face detector");
        }
        v.activation();
      }
     
      ros::spinOnce();
      loop_rate.sleep();
    
    }
    cv::destroyWindow("CamP300");
    free(v.pls);
}