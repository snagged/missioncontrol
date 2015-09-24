#ifndef WAVESIMULATION_H
#define WAVESIMULATION_H

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>


namespace gazebo
{
  class WaveSimulation : public ModelPlugin {
  
  public:
    WaveSimulation() {}
    ~WaveSimulation(){}
    
    /// \brief loads the plugin - inserts all params to assigned variables etc.
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    
    /// \brief updates the force on link
    virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

private:
 


private:
  
  
    // Pointer to the model
    private: physics::ModelPtr model;
    private: physics::LinkPtr link;
    private: physics::WorldPtr world;

    private: event::ConnectionPtr updateConnection;
    event::ConnectionPtr update_event_;
    
    bool debug;

    

};

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(WaveSimulation)


}

#endif
