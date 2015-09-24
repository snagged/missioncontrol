#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <ros/ros.h>
#include <relative_force_engine/relative_force_engine.h>

namespace gazebo
{

    void RelativeForcePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
	
      this->model = _parent;
      this->world = model->GetWorld();
      
      
      
      if (_sdf->HasElement("link"))
	this->link = this->model->GetLink(_sdf->Get<std::string>("link"));

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
	  boost::bind(&RelativeForcePlugin::OnUpdate, this, _1));
      
      
      // register ROS nodes
      rosnode_ = new ros::NodeHandle("gazebo/relative_force_engine");

      // initialize subscriber to water current
      ros::SubscribeOptions ops = ros::SubscribeOptions::create<relative_force_engine::object_params>(
		  model->GetName(), 1,
		  boost::bind(&RelativeForcePlugin::SetParamsCallBack, this, _1),
		  ros::VoidPtr(), &callback_queue_);
      body_force_subscriber_ = rosnode_->subscribe(ops);

    }

    // Called by the world update start event
    void RelativeForcePlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      //call updates from ros message.
      callback_queue_.callAvailable();
      
      //add force and torque if duration has'nt expired.
      math::Vector3 f = this->link->GetWorldForce();
      math::Vector3 t = this->link->GetWorldTorque();

      if (world->GetSimTime().Double() < force_expiration_secs)
      {
	this->link->AddRelativeForce(this->force);
      }
      else
      {
	this->link->AddRelativeForce(math::Vector3(0,0,0));
	force_expiration_secs = 0;
      }

      if (world->GetSimTime().Double() < torque_expiration_secs)
      {
	link->AddRelativeTorque(this->torque);
      }
      else
      {
	this->link->AddRelativeTorque(math::Vector3(0,0,0));
	torque_expiration_secs = 0;
      }      
    
    }

    
    void RelativeForcePlugin::SetParamsCallBack (const relative_force_engine::object_paramsConstPtr &_params)
    {
      //set force flag
      if (_params->command_types % 2 == 1)
      {
	this->force = math::Vector3(_params->force.x, _params->force.y, _params->force.z);
	force_expiration_secs = this->world->GetSimTime().Double() + _params->force_duration;
      }
      
      //set torque flag
      if ((_params->command_types / 2) % 2 == 1)
      {
	this->torque = math::Vector3(_params->torque.x, _params->torque.y, _params->torque.z);
	torque_expiration_secs = this->world->GetSimTime().Double() + _params->torque_duration;
      }
    }
    

  };


  
