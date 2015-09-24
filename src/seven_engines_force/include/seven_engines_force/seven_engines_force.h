#ifndef FREEFLOATINGGAZEBOFLUID_H
#define FREEFLOATINGGAZEBOFLUID_H

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

#include <seven_engines_force/engine_forces.h>


namespace gazebo
{
  class SevenEngineForce : public ModelPlugin {
  
  public:
    SevenEngineForce() {}
    ~SevenEngineForce()
    {
        rosnode_->shutdown();
        delete rosnode_;
	delete[] engine_forces;
    }
    
    /// \brief loads the plugin - inserts all params to assigned variables etc.
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    
    /// \brief updates the force on link
    virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

private:

    /// \brief sets new forces to engines.
    void SetParamsCallBack (const seven_engines_force::engine_forcesConstPtr &_params);
    
    /// \brief resets all engine forces to zero
    void resetEngineForces();
    
    /// \brief apply force according to engine forces.
    math::Vector3 getForce();
    
    /// \brief apply torque according to engine torques
    math::Vector3 getTorque();
    
    


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
    
    
    //calculation forces
    double *engine_forces;
    double rx, ry, rz, rroll;
    double forceCoef;
    double next_time_out;
    double default_command_timeout_period;
    bool isRPM;
    math::Vector3 lastForce, lastTorque;
    
    bool debug;

    

    // subscriber to fluid velocity (defined in the world frame)
    ros::Subscriber body_force_subscriber_;
    
};

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SevenEngineForce)  


}

#endif // FREEFLOATINGGAZEBOFLUID_H
