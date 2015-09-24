#include "ros/ros.h"

#include <opencv2/opencv.hpp>

#include "gazebo_dll_controller/ros_read_write.h"
#include "gazebo_msgs/GetModelState.h"
#include "propeller_engine/engine_msg.h"
#include "seven_engines_force/engine_forces.h"
#include "relative_force_engine/object_params.h"
#include "basic_sonar/sonar_serv.h"
#include "basic_sonar/sonar_image_serv.h"
#include "gazebo_dll_controller/command.h"

#include <string>
#include <vector>
#include <boost/thread/mutex.hpp>

#define HYDROCAMEL_COMMAND_TOPIC "/gazebo/hydrocamel/command"

using namespace std;


  void RosCommunicator::initCommunication(int argc, char ** argv){
      ros::init(argc, argv, "submarines_communication");
      n = new ros::NodeHandle();
  }

  
  RosCommunicator::RosCommunicator() {
      throw 99;    //static class.
  }
  
  void RosCommunicator::updateLastCommandMessage(const gazebo_dll_controller::command::ConstPtr& msg)
  {
    last_message.cmdnum = msg->cmdnum;
    last_message.x = msg->x;
    last_message.y = msg->y;
    last_message.z = msg->z;
    last_message.anglex = msg->anglex;
    last_message.angley = msg->angley;
    last_message.anglez = msg->anglez;
  }
   
  
  
  void RosCommunicator::openPublishers(const char *bluefin_topic, const char *hydrocamel_topic, const char *kayak_topic, const char *hydro_command_topic) {
    RosCommunicator::initCommunication(0, 0);
    bluefin_pub = n->advertise<propeller_engine::engine_msg>( std::string(bluefin_topic), 10);
    hydrocamel_pub = n->advertise<seven_engines_force::engine_forces>( std::string(hydrocamel_topic), 10);
    if (kayak_topic != 0)
    {
      kayak_pub = n->advertise<relative_force_engine::object_params>(std::string(kayak_topic), 10);
      
      while (bluefin_pub.getNumSubscribers()*hydrocamel_pub.getNumSubscribers()*kayak_pub.getNumSubscribers() == 0 )
	sleep(0.001);
    }
    else
    {
       while (bluefin_pub.getNumSubscribers()*hydrocamel_pub.getNumSubscribers() == 0 )
	sleep(0.001);
    }
    
    if (hydro_command_topic != 0)
    {
      hydrocamel_command_sub = n->subscribe(hydro_command_topic, 5, RosCommunicator::updateLastCommandMessage);
    }
    

  }
 
  double * RosCommunicator::getModelPose(const char * model, const char * relative_pose_param){
    ros::ServiceClient client = n->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState req;
    req.request.model_name = model;
    req.request.relative_entity_name = relative_pose_param;
    
    client.call(req);
    double *response = new double[7];
    response[0] = req.response.pose.position.x;
    response[1] = req.response.pose.position.y;
    response[2] = req.response.pose.position.z;
    response[3] = req.response.pose.orientation.x;
    response[4] = req.response.pose.orientation.y;
    response[5] = req.response.pose.orientation.z;
    response[6] = req.response.pose.orientation.w;
    
    //ROS_INFO("%f %f %f %f %f %f", response[0], response[1], response[2], response[3], response[4], response[5], response[6]);
    
    return (response);
  }

  double * RosCommunicator::getModelTwist(const char * model, const char * relative_pose_param){
       
    ros::ServiceClient client = n->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState req;
    req.request.model_name = model;
    req.request.relative_entity_name = relative_pose_param;
    
    client.call(req);
    
    double * response = new double[6];
			  
    response[0] = req.response.twist.linear.x;
    response[1] = req.response.twist.linear.y;
    response[2] = req.response.twist.linear.z;
    response[3] = req.response.twist.angular.x;
    response[4] = req.response.twist.angular.y;
    response[5] = req.response.twist.angular.z;

    return (response);
    
  }
  
  void RosCommunicator::publishBluefinEngineMsg(double force, double x_rotation, double y_rotation){
    propeller_engine::engine_msg msg;
    msg.force = force;
    msg.l_angle = x_rotation;
    msg.u_angle = y_rotation;
    
    bluefin_pub.publish(msg);
    
    ros::spinOnce();  
  }

  void RosCommunicator::publishHydroCamelEngineMsg(double force1, double force2, double force3, double force4, double force5, double force6, double force7){
    seven_engines_force::engine_forces msg;
    msg.engine_forces[0] = force1;
    msg.engine_forces[1] = force2;
    msg.engine_forces[2] = force3;
    msg.engine_forces[3] = force4;
    msg.engine_forces[4] = force5;
    msg.engine_forces[5] = force6;
    msg.engine_forces[6] = force7;

    hydrocamel_pub.publish(msg);
    
  }
  
  std::vector<float> RosCommunicator::getSonarScan(const char* sonar_name)
  {
    std::string service_path("/gazebo/basic_sonar/");
    ros::ServiceClient client = n->serviceClient<basic_sonar::sonar_serv>(service_path.append(sonar_name).c_str());
    basic_sonar::sonar_serv req;
    client.call(req);
    return req.response.res;
  }
  
  int RosCommunicator::getSonarArraySize(const char* sonar_name)
  {
    std::vector<float> vec = getSonarScan(sonar_name);
    return vec.size();
  }
  
  
  float* RosCommunicator::getSonarScanArray(const char* sonar_name)
  {
    std::vector<float> vec = getSonarScan(sonar_name);
    float* arr = new float[vec.size()];
    for (int i = 0; i < (int) vec.size(); i++){
      *(arr + i) = vec.at(i);
    }  
    return arr;
  }  
  
  
  void RosCommunicator::getCommandMessageParameters(int *cmdnum, double *pitch_target, double *roll_target, double *yaw_target, double *x_target, double *y_target, double *depth_target)
  {   
    ros::spinOnce();
    *cmdnum = last_message.cmdnum;
    *pitch_target = last_message.angley; 
    *roll_target = last_message.anglex;
    *yaw_target = last_message.anglez;
    *x_target = last_message.x;
    *y_target = last_message.y;
    *depth_target = last_message.z;
    
    return;
  }
  
  void RosCommunicator::publishKayakEngineMsg(double force_x, double force_y, double force_z, 
				      double torque_x, double torque_y, double torque_z, double force_duration, double torque_duration){
    relative_force_engine::object_params msg;
    
    msg.force.x = force_x;
    msg.force.y = force_y;
    msg.force.z = force_z;
    msg.torque.x = torque_x;
    msg.torque.y = torque_y;
    msg.torque.z = torque_z;
    msg.force_duration = force_duration;
    msg.torque_duration = torque_duration;
    
    msg.command_types = 3;

    kayak_pub.publish(msg);
    
  }
  
  
  std::vector<float> RosCommunicator::createSonarImage(const char* sonar_name, const char* path)
  {
    std::vector<float> scan = getSonarScan(sonar_name);
    if (scan.size() == 0){
      return scan;
    }
    //picture should be a perfect square.
    int lineSize = (int) std::pow(scan.size(),0.5);
    
    // Create mat and insert the vector data.
    cv::Mat mat(lineSize, lineSize, CV_32FC1, &(scan[0]) );
    
    //flip image by 270 degrees to fit look above.
    cv::Point2f src_center(mat.cols/2.0F, mat.rows/2.0F);
    cv::Mat rot_mat = cv::getRotationMatrix2D(src_center, 90, 1.0);
    cv::Mat dst;
    cv::warpAffine(mat, dst, rot_mat, mat.size());

    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    
    //save mat to file
    imwrite(path, dst, compression_params);
    return scan;
  }
  
