#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <algorithm>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

ros::Publisher publisher_cnn1;
ros::Publisher publisher_cnn2;
ros::Publisher publisher_svm1;
ros::Publisher publisher_svm2;

//resends topic as one detection
void resend_cnn_1( const geometry_msgs::PoseArray::ConstPtr& msg)
{  
  if (msg->poses.size()>0) {
    geometry_msgs::PoseArray nene;
    geometry_msgs::Pose ps;
    ps= msg->poses[0];
    nene.poses.push_back(ps);
    nene.header = msg->header;
    publisher_cnn1.publish(nene);
  }
}

void resend_svm_1( const geometry_msgs::PoseArray::ConstPtr& msg)
{
  if (sizeof(msg->poses)>0){
    geometry_msgs::PoseArray nene;
    geometry_msgs::Pose ps;
    ps= msg->poses[0];
    nene.poses.push_back(ps);
    nene.header = msg->header;
    publisher_svm1.publish(nene);
  }
}

//resends detections on as much as 2 different topic
void resend_cnn_2( const geometry_msgs::PoseArray::ConstPtr& msg)
{
  for(int i = 0; i<sizeof(msg->poses);i++){
    geometry_msgs::PoseArray nene;
    geometry_msgs::Pose ps;
    ps= msg->poses[i];
    nene.poses.push_back(ps);
    nene.header = msg->header;
    if (msg->poses[i].position.y>0){
      publisher_cnn1.publish(nene);
    }else{
      publisher_cnn2.publish(nene);
    }
  }
}
void resend_svm_2( const geometry_msgs::PoseArray::ConstPtr& msg)
{
  for(int i = 0; i<sizeof(msg->poses);i++){
    geometry_msgs::PoseArray nene;
    geometry_msgs::Pose ps;
    ps= msg->poses[i];
    nene.poses.push_back(ps);
    nene.header = msg->header;
    if (msg->poses[i].position.y>0){
      publisher_svm1.publish(nene);
    }else{
      publisher_svm2.publish(nene);
    }
  }
}


int main(int argc, char** argv)
{


  // Initialize node and publisher.
  ros::init( argc, argv, "ppl_spliter");
  
  ros::NodeHandle private_node_handle_("~");

  //Get params
  std::string single;
  private_node_handle_.param<std::string>("solo", single, "true");
  std::string svmTopic;
  private_node_handle_.param<std::string>("svmTopic", svmTopic, "/radar_detector_ol/poses");
  std::string cnnTopic;
  private_node_handle_.param<std::string>("cnnTopic", cnnTopic, "/deep_radar/out/clustering");


  //make subscrbers and publishers according to settings
  publisher_cnn1 =  private_node_handle_.advertise<geometry_msgs::PoseArray>("/pt/cnn1", 1, true);
  publisher_svm1 =  private_node_handle_.advertise<geometry_msgs::PoseArray>("/pt/svm1", 1, true);
  ros::Subscriber sub_1;
  ros::Subscriber sub_2;
  if (single=="true")
    {
      std::cout << "running solo spliter" <<std::endl;
      sub_1 =  private_node_handle_.subscribe<geometry_msgs::PoseArray>(cnnTopic, 10, resend_cnn_1);
      sub_2 =  private_node_handle_.subscribe<geometry_msgs::PoseArray>(svmTopic, 10, resend_svm_1);
     }else{
      std::cout << "runing duo spliter" <<std::endl;
      publisher_cnn2 =  private_node_handle_.advertise<geometry_msgs::PoseArray>("/pt/cnn2", 1, true);
      publisher_svm2 =  private_node_handle_.advertise<geometry_msgs::PoseArray>("/pt/svm2", 1, true);
      sub_1 =  private_node_handle_.subscribe<geometry_msgs::PoseArray>(cnnTopic, 1, resend_cnn_2);
      sub_2 =  private_node_handle_.subscribe<geometry_msgs::PoseArray>(svmTopic, 1, resend_svm_2);
  }
  
  ros::spin();

  return 0;
}



