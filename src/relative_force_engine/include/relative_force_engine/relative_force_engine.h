#ifndef FREEFLOATINGGAZEBOFLUID_H
#define FREEFLOATINGGAZEBOFLUID_H

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

#include <relative_force_engine/object_params.h>


namespace gazebo
{
  class RelativeForcePlugin : public ModelPlugin {
  
  public:
    RelativeForcePlugin() {}
    ~RelativeForcePlugin()
    {
        rosnode_->shutdown();
        delete rosnode_;
    }

    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

private:

    void SetParamsCallBack (const relative_force_engine::object_paramsConstPtr &_params);
    
    


private:
  
  
      // Pointer to the model
    private: physics::ModelPtr model;
    private: physics::LinkPtr link;
    private: physics::WorldPtr world;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    
    // general data
    ros::NodeHandle* rosnode_;
    ros::CallbackQueue callback_queue_;
    physics::WorldPtr world_;
    event::ConnectionPtr update_event_;
    
    double force_expiration_secs, torque_expiration_secs;
    
    math::Vector3 force;
    math::Vector3 torque;

    

    // subscriber to fluid velocity (defined in the world frame)
    ros::Subscriber body_force_subscriber_;
    
};

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RelativeForcePlugin)  


}

#endif // FREEFLOATINGGAZEBOFLUID_H
