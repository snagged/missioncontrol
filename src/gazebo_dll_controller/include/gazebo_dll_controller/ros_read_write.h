#include "gazebo_dll_controller/command.h"


class RosCommunicator {
 
private:
  ///\brief ititiates the ros node. This function should be called before any other function.
  static void initCommunication(int argc, char ** argv);
  

public :
  
  typedef struct command_message {
    int cmdnum;
    double x;
    double y;
    double z;
    double anglex;
    double angley;
    double anglez;
  } command_message;
  
  
  static ros::NodeHandle *n;
  static ros::Publisher bluefin_pub;
  static ros::Publisher hydrocamel_pub;
  static ros::Publisher kayak_pub;
  static ros::Subscriber hydrocamel_command_sub;
  static command_message last_message;

  
  RosCommunicator();
   
  
  ///\brief ititiates the ros publishers. This function should be called after initialization, and before publishing to ros.
  static void openPublishers(const char *bluefin_topic, const char *hydrocamel_topic, const char *kayak_topic = 0, const char *hydro_command_topic = 0);

  
  ///\brief retrieves the pose of a model.
  ///\param model
  ///\param relative_pose_param the service returns the pose relatively to the model given in this param.
  ///\return a pointer to 7 doubles {x_pose, y_pose, z_pose, x_orientation, y_orientation, z_orientation, w_orientation}.
  static double * getModelPose(const char * model, const char * relative_pose_param="");

  
  ///\brief retrieves the linear and angular velocity of a model.
  ///\param model
  ///\param relative_pose_param the service returns the pose relatively to the model given in this param.
  ///\return a pointer to 7 doubles {x_linear, y_linear, z_linear, x_angular, y_angular, z_angular}.
  static double * getModelTwist(const char * model, const char * relative_pose_param="");

  
  ///\brief publishes a message to the bluefin topic
  ///\param force
  ///\param x_rotation
  ///\param y_rotation
  static void publishBluefinEngineMsg(double force, double x_rotation, double y_rotation);

  
  ///\brief publishes a message to the HydroCamel topic, that gets 7 forces for the seven engines.
  static void publishHydroCamelEngineMsg(double force1, double force2, double force3, double force4, double force5, double force6, double force7);

  
  ///\brief publishes a message to the Kayak topic, gets force, torque, force_duration, torque_duration
  static void publishKayakEngineMsg(double force_x, double force_y, double force_z, 
				    double torque_x, double torque_y, double torque_z, double force_duration, double torque_duration);
  
  ///\brief retrieves sonar scan from sonar to a vector.
  static std::vector<float> getSonarScan(const char* sonar_name);
  
  ///\brief retrieves sonar scan from sonar, and inserts it to an array.
  static float* getSonarScanArray(const char* sonar_name);
  
  ///\brief returns the size of a scanned array.
  static int getSonarArraySize(const char* sonar_name);
  
  ///\brief creates an image of the last sonar scan in path. flips the image by 270 degrees.
  ///\param path the path to create on.
  static std::vector<float> createSonarImage(const char* sonar_name, const char* path);
  
  ///\brief retrieves command parameters for hydro camel.
  static void getCommandMessageParameters(int *cmdnum, double *pitch_target, double *roll_target, double *yaw_target, double *x_target, double *y_target, double *depth_target);
  
  static void updateLastCommandMessage(const gazebo_dll_controller::command::ConstPtr& msg);

};



ros::NodeHandle *RosCommunicator::n = 0;
ros::Publisher RosCommunicator::bluefin_pub;
ros::Publisher RosCommunicator::hydrocamel_pub;
ros::Publisher RosCommunicator::kayak_pub;
ros::Subscriber RosCommunicator::hydrocamel_command_sub;
RosCommunicator::command_message RosCommunicator::last_message;
