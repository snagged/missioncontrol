#ifndef FREEFLOATINGGAZEBOFLUID_H
#define FREEFLOATINGGAZEBOFLUID_H

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

#include <seven_engines_force/engine_forces.h>
#include <propeller_engine/engine_msg.h> 

#include <string>

namespace gazebo
{

class GazeboDllControllerPlugin : public  WorldPlugin
{
public:
    GazeboDllControllerPlugin() {}
    ~GazeboDllControllerPlugin()
    {
    }

    virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
    virtual void Update();


private:
    // plugin options
    physics::WorldPtr world_;
    event::ConnectionPtr update_event_;
    
    
    // subscriber to fluid velocity (defined in the world frame)
    ros::Publisher bluefin_publisher;
    ros::Publisher hydrocamel_publisher;
    
    bool move_bluefin;
    /*physics::ModelPtr bluefin_model;
    physics::LinkPtr bluefin_engine;
    physics::JointPtr bluefin_engine_joint;
    std::string bluefin_dll;
    
    bool move_hydrocamel;
    physics::ModelPtr hydrocamel_model;
    physics::LinkPtr hydrocamel_engine;
    physics::JointPtr bluefin_engine_joint;
    std::string bluefin_dll;*/
  
};

GZ_REGISTER_WORLD_PLUGIN(GazeboDllControllerPlugin)
}
#endif // FREEFLOATINGGAZEBOFLUID_H
