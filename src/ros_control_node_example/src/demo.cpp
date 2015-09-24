#include "ros/ros.h"
#include "ros_read_write.cpp"
#include <cmath>
#include "stdio.h"

#define HOLD_POSITION 1
#define MOVE_TO_POINT 2
#define PI 3.14159265359
///\brief Example code. You may insert you're own code here as long as you remember to leave the first two lines.



  double delta_z, delta_pitch, delta_roll, delta_yaw,pitch_target,roll_target,yaw_target;
  double depth_target, x_target, y_target, delta_y, delta_x, yaw,roll,pitch;
  double FL,FR,YF,YR,PF,WL,WR;
  int cmdnum;  //cmdnum=1: hold position, cmdnum=2: move to point;



void waypointCallback(const gazebo_dll_controller::command msg){
	x_target=msg.x;
	y_target=msg.y;
	depth_target=msg.z;
	roll_target=msg.anglex;
	yaw_target=msg.angley;
	pitch_target=msg.anglez;
	std::cout << "New waypoint callback function got called!" << std::endl;
}



int main(int argc, char **argv)
{
  RosCommunicator::openPublishers("/gazebo/propeller_engine/bluefin" , "/gazebo/seven_engines_force/Hydro_Camel"
    ,"/gazebo/relative_force_engine/KAYAK", "/newWaypoint");


double sleep_time = 0.5;  
  cmdnum = MOVE_TO_POINT;
  pitch_target = 0;
  roll_target = 0;
  yaw_target = 0;
  x_target = 200;
  y_target = 200;
  depth_target = -5;
  
  ros::init(argc, argv, "waypoint_subscriber");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("newWaypoint", 1000, waypointCallback);
  ros::spinOnce();

  while (ros::ok())
  {
    
    double *pose;   //x y z roll pitch yaw
    double *twist;
    RosCommunicator::getCommandMessageParameters(&cmdnum, &pitch_target, &roll_target, &yaw_target, &x_target, &y_target, &depth_target);
    
    int test = 1;
    pose = RosCommunicator::getModelPose("Hydro_Camel");
    twist = RosCommunicator::getModelTwist("Hydro_Camel");
    std::cout << "Live = " << test++ << std::endl; 
    roll = atan2(2*(pose[3]*pose[6] + pose[4]*pose[5]),1-2*(pose[5]*pose[5]+pose[6]*pose[6]));
    if (roll < 0)
	roll = -PI - roll;
    else
	roll = PI - roll;
    yaw = atan2(2*(pose[3]*pose[4] + pose[5]*pose[6]),1-2*(pose[4]*pose[4]+pose[5]*pose[5]));
    pitch = -asin(2*(pose[3]*pose[5] - pose[6]*pose[4]));
    std::cout << "pitch = " << pitch << std::endl; 
    std::cout << "roll = " << roll << std::endl;
    std::cout << "yaw = " << yaw << std::endl; 
    //logic here:
    /*if (pose[0] >= 70 || (pose[0]>40 && twist[0] < 0.1))
      RosCommunicator::publishBluefinEngineMsg(-500, -0.1, 0);
    else
      RosCommunicator::publishBluefinEngineMsg(500, 0.1, 0);
    
    RosCommunicator::publishHydroCamelEngineMsg(401,0,0,0,0,0,0);
    
    RosCommunicator::publishKayakEngineMsg(50,0,0,0,0,20, 0.5,0.5);
  
    ros::spinOnce();*/
    if (cmdnum == HOLD_POSITION){
    	delta_z = depth_target*10;//(depth_target - pose[2])*10;
    	delta_roll = roll_target - roll;
    	delta_pitch = (pitch_target - pitch)/10;
	delta_yaw = yaw_target - yaw;
        delta_x = x_target;// - pose[0])*10;
	delta_y = y_target;// - pose[1])*10;
        FL = -delta_x;
        FR = -delta_x;
        YF = -delta_y + delta_yaw;
	YR = -delta_y - delta_yaw;
	PF = delta_z + delta_pitch;
        WL = (delta_z - delta_pitch)/2;
	WR = (delta_z - delta_pitch)/2;
	std::cout << "pitch = " << delta_pitch*180/3.14 << std::endl; 
        std::cout << "delta_x = " << delta_x << std::endl;
	std::cout << "delta_y = " << delta_y << std::endl; 
	std::cout << "depth = " << pose[2] << std::endl; 
        std::cout << "delta yaw = " << delta_yaw << std::endl; 
    }
    if (cmdnum == MOVE_TO_POINT){
	double dx,dy,xtag,ytag;
	dx = x_target - pose[0];
	dy = y_target - pose[1]; 
	std::cout << "dx = " << dx << std::endl; 
	std::cout << "dy = " << dy << std::endl;
	if (abs(dx) < 1 && abs(dy) < 1){
		std::cout << "Small Error" << std::endl; 
		FL = 0;
		FR = 0;
		YF = 0;
		YR = 0;
		PF = delta_z - delta_pitch;
		WL = (delta_z + delta_pitch)/2;
		WR = (delta_z + delta_pitch)/2;	
	}
	else{
		std::cout << "BIG Error" << std::endl; 
		delta_z = (depth_target - pose[2])*10;
	    	delta_roll = roll_target - roll;
	    	delta_pitch = (pitch_target - pitch)/10;
		yaw += PI;
		if (yaw > PI)
			yaw -= 2*PI;
		if (yaw < -PI)
			yaw += 2*PI;
		xtag = (dx)*cos(yaw) + (dy)*sin(yaw);
		ytag = -(dx)*sin(yaw) + (dy)*cos(yaw);			
		if (xtag == 0)	
			if (ytag < 0)
				yaw_target = -PI/2;
			else
				yaw_target = PI/2;
		else{				
			yaw_target = atan(ytag/xtag);	
			if (xtag < 0)
				yaw_target -= PI;
			if (yaw_target > PI)
				yaw_target -= 2*PI;
			if (yaw_target < -PI)
				yaw_target += 2*PI;	
		}
		if (yaw_target < 0.05)
			delta_yaw = (yaw_target)*5;		
		else		
			delta_yaw = (yaw_target)*15;
		FL = -75 + delta_yaw;
		FR = -75 - delta_yaw;
		YF = 0;
		YR = 0;
		PF = delta_z + delta_pitch;
		WL = (delta_z - delta_pitch)/2;
		WR = (delta_z - delta_pitch)/2;
		//std::cout << "pitch = " << delta_pitch*180/3.14 << std::endl; 
		//std::cout << "roll = " << delta_roll*180/3.14 << std::endl; 		
		std::cout << "delta_x = " << dx << std::endl;
		std::cout << "delta_y = " << dy << std::endl; 
		std::cout << "yaw = " << yaw*180/PI << std::endl;
		std::cout << "x = " << pose[0] << std::endl;
		std::cout << "y = " << pose[1] << std::endl;
	        //std::cout << "cos(yaw) = " << cos(yaw) << std::endl;
		std::cout << "xtag = " << xtag << std::endl; 
		std::cout << "ytag = " << ytag << std::endl; 
		std::cout << "yaw_target = " << yaw_target*180/PI << std::endl; 
				
	}
    }
    RosCommunicator::publishHydroCamelEngineMsg(FL,FR,YF,YR,PF,WL,WR);
    sleep(sleep_time);
    delete pose;
    delete twist;
  }

}
