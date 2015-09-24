#include "ros/ros.h"
#include "ros_read_write.cpp"
#include <std_msgs/String.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
gazebo_dll_controller::command msg;

geometry_msgs::Point modelmsg;
ros::Publisher pu1;
ros::Publisher pu2;

void waypointCallback(const std_msgs::String array){

	int tmp11 = (int) (array.data[0])-48;
	int tmp12 = (int) (array.data[1])-48;
	int tmp13 = (int) (array.data[2])-48;
	int tmp21 = (int) (array.data[4])-48;
	int tmp22 = (int) (array.data[5])-48;
	int tmp23 = (int) (array.data[6])-48;
	int tmp31 = (int) (array.data[8])-48;
	int tmp32 = (int) (array.data[9])-48;
	int tmp33 = (int) (array.data[10])-48;
	std::cout << tmp11*100+tmp12*10+tmp13-500 <<std::endl;
	msg.x=tmp11*100+tmp12*10+tmp13-500;
	msg.y=tmp21*100+tmp22*10+tmp23-500;
	msg.z=tmp31*100+tmp32*10+tmp33-500;
	//msg.rolltarget=array.data[3];
	//msg.yawtarget=array.data[4];
	//msg.pitchtarget=array.data[5];
	//msg.command_types=array.data[6];
	msg.cmdnum=2;
	std::cout << "msg_relay: array received on /newWaypointMsg, publishing command_msg on /newWaypoint" << std::endl;
	pu1.publish(msg);
	ros::spinOnce();
}

void modelstateCallback(const gazebo_msgs::ModelStates states){
	ros::Rate loop_rate(1);
	modelmsg.x=states.pose[3].position.x;
	modelmsg.y=states.pose[3].position.y;
	modelmsg.z=states.pose[3].position.z;
	std::cout << "msg_relay: model state received on /gazebo/model_states, publishing std_msg on /modelstate" << std::endl;
	pu2.publish(modelmsg);
	ros::spinOnce();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "msg_relay");
  ros::NodeHandle n;
  ros::Publisher pub1 = n.advertise<gazebo_dll_controller::command>("newWaypoint", 1000);
  pu1=pub1;
  ros::Publisher pub2 = n.advertise<geometry_msgs::Point>("modelstate", 1000);
  pu2=pub2;
  ros::Subscriber sub = n.subscribe("newWaypointMsg", 1000, waypointCallback);
  ros::Subscriber modelsub = n.subscribe("gazebo/model_states", 1000, modelstateCallback);

  ros::Rate loop_rate(1);

  ros::spin();
}

