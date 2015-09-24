#ifndef PROPELLER_ENGINE_H
#define PROPELLER_ENGINE_H

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>

#include <propeller_engine/engine_msg.h> 


namespace gazebo
{
  ///\brief The PropellerEngine class. a class that simulates a body with a link that is suppose to be the engine.
  // 	the plugin applies force from the given link, and sets the angle of the joints axes to simulate the ungine spin.
  // the XML snippet of the plugin is configured as followed:
  //       <plugin name="propeller_engine" filename="libpropeller_engine.so">
  //       <descriptionParam>robot_description</descriptionParam>
  //       <link>##LINK_THAT_APPLIES_FORCE##</link> <!-- usually the child link of the joint -->
  //       <joint>##JOINT_FOR_AXES_CHANGE##</joint>
  //     </plugin>
  // optional additional tags:
  //	<timeout></timeout> - timeout of latest message. default is no timeout.
  //	<reset_propeller></reset_propeller> - if true - after timeout reset propeller. default is false.
  //	<debug></debug> - if true - presents debug messages. default is false.
  class PropellerEngine : public ModelPlugin {
  
  public:
    PropellerEngine() {}
    ~PropellerEngine()
    {
        rosnode_->shutdown();
        delete rosnode_;
    }

    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    virtual void OnUpdate(const common::UpdateInfo & /*_info*/);
    
    static const math::Vector3 Y_AXIS;

private:

    void SetParamsCallBack (const propeller_engine::engine_msgConstPtr &_params);
  
  
      // Pointer to the model
    physics::ModelPtr model;
    physics::LinkPtr link;
    physics::JointPtr joint;
    physics::WorldPtr world;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;
    
    ///\brief Rotates a vector around X axis.
    math::Vector3 RotateVectorAroundXAxis(const math::Vector3 &vec, double angle);
    
    /// \brief get the rotation angle for the axis according to the formula size = sqrt(x^2+y^2).
    double GetRotationSize(double x_rotation, double y_rotation);
    
    /// \brief get the rotation axis from the two rotation angles, according to the formulat axis=tan(y/x).
    double GetAxisAngle(double x_rotation, double y_rotation);    
    
    //parameters of force:
    math::Vector3 current_axis;
    double angle;
    double force;
    bool has_changed;
    double timeout;
    double next_reset;
    bool debug;
    bool reset_propeller;
    
    
    
    // general data
    ros::NodeHandle* rosnode_;
    ros::CallbackQueue callback_queue_;
    physics::WorldPtr world_;
    event::ConnectionPtr update_event_;

    

    // subscriber to fluid velocity (defined in the world frame)
    ros::Subscriber body_force_subscriber_;
    
};

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PropellerEngine);
  
  const math::Vector3 PropellerEngine::Y_AXIS(0,0,1);

}

#endif // PROPELLER_ENGINE_H
