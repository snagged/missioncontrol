#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <ros/ros.h>
#include <propeller_engine/propeller_engine.h>
#include <math.h>


namespace gazebo
{
      
  
  
    void PropellerEngine::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      
      this->model = _parent;
      this->world = model->GetWorld();
      has_changed = false;
      this->timeout = -1;
      this->next_reset = -1;
      this->debug = false;
      this->reset_propeller = false;
      
      
      if (_sdf->HasElement("debug"))
	this->debug = _sdf->Get<bool>("debug");
      
      if (_sdf->HasElement("link"))
	this->link = this->model->GetLink(_sdf->Get<std::string>("link"));
      if (_sdf->HasElement("joint"))
	this->joint = this->model->GetJoint(_sdf->Get<std::string>("joint"));
      if (_sdf->HasElement("timeout"))
	this->timeout = _sdf->Get<double>("timeout");
      if (_sdf->HasElement("reset_propeller"))
	this->reset_propeller = _sdf->Get<bool>("reset_propeller");      

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
	  boost::bind(&PropellerEngine::OnUpdate, this, _1));
      
      
      // register ROS nodes
      rosnode_ = new ros::NodeHandle("gazebo/propeller_engine");
      ros::SubscribeOptions ops = ros::SubscribeOptions::create<propeller_engine::engine_msg>(
		  model->GetName(), 1,
		  boost::bind(&PropellerEngine::SetParamsCallBack, this, _1),
		  ros::VoidPtr(), &callback_queue_);
      body_force_subscriber_ = rosnode_->subscribe(ops);
      
      this->joint->SetAxis(0, Y_AXIS);
      current_axis = math::Vector3(0,0,0);
      angle = 0;
      force = 0;    

    }

    // Called by the world update start event
    void PropellerEngine::OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      //call updates from ros message.
      callback_queue_.callAvailable();
      
      if ((this->timeout != -1) && (this->world->GetRealTime().Double() > this->next_reset))
      {
	if (force != 0)
	{
	  if (this->debug)
	    ROS_DEBUG("reset force to 0");
	  if (this->debug && this->reset_propeller)
	    ROS_DEBUG("reset angle to 0");
	}
	
	force = 0;
	
	if (this->reset_propeller)
	  angle = 0;

      }
      
      if (has_changed)
      {
	this->joint->SetAngle(0, math::Angle(this->joint->GetAngle(0).Radian()*-1));
	this->joint->SetAngle(0, math::Angle(0));
	this->joint->SetAxis(0, current_axis);
	has_changed=false;
	
	if (this->debug)
	  ROS_DEBUG("new force message: angle-%f force-%f current_axis-(%f,%f,%f)", angle, force, current_axis.x, current_axis.y, current_axis.z);
      }
      
      this->joint->SetAngle(0, math::Angle(angle));
      this->link->AddRelativeForce(math::Vector3(force,0,0));
      
    
      
    
    }

    
    void PropellerEngine::SetParamsCallBack (const propeller_engine::engine_msgConstPtr &_params)
    {
      double axis_angle = GetAxisAngle(_params->l_angle, _params->u_angle); 
      current_axis = RotateVectorAroundXAxis(Y_AXIS, axis_angle);
      
      angle = GetRotationSize(_params->l_angle, _params->u_angle);
      
      force = _params->force;
      has_changed = true;
      if (this->timeout != -1)
      {
	this->next_reset = this->world->GetRealTime().Double() + this->timeout;
      }
      
      	if (this->debug)
	  ROS_DEBUG("set force: angle-%f force-%f current_axis-(%f,%f,%f) next_reset=%f", angle, force, current_axis.x, current_axis.y, current_axis.z, next_reset);
    }
    
    
    math::Vector3 PropellerEngine::RotateVectorAroundXAxis(const math::Vector3 &vec, double angle)
    {
      math::Vector3 ans;
      
      ans.x = vec.x;
      ans.y = std::cos(angle)*vec.y - std::sin(angle)*vec.z;
      ans.z = std::sin(angle)*vec.y + std::cos(angle)*vec.z;
      
      return ans;
      
    }
    
    double PropellerEngine::GetRotationSize(double x_rotation, double y_rotation)
    {
      return pow( std::pow(x_rotation, 2) + std::pow(y_rotation, 2) , 0.5);
    }
    
    
    double PropellerEngine::GetAxisAngle(double x_rotation, double y_rotation)
    {
      if (x_rotation == 0)
      {
	return (y_rotation >= 0) ? (M_PI / 2) : 3*(M_PI / 2);
      }
      else
      {
	if (x_rotation > 0 && y_rotation >= 0)
	{
	  return std::atan(y_rotation / x_rotation);
	}
	
	else if (x_rotation > 0 && y_rotation < 0)
	{
	  return std::atan(y_rotation / x_rotation) + 2*M_PI;
	}
	
	else //x_rotation < 0
	{
	  return std::atan(y_rotation / x_rotation) + M_PI;
	}
      }
      
    }
    

  };


  
