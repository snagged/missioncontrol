#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <ros/ros.h>
#include <seven_engines_force/seven_engines_force.h>


/*
 *   <plugin name="seven_forces_engine" filename="libseven_engines_force.so">
 *     <link>bluefin_body</link>
 *     <rx>1</rx> 
 *     <ry>1</ry> 
 *     <rz>1</rz>
 *     <rroll>2</rroll>
 *
 *     <!-- optional:
 *     <forceCoef>1</forceCoef>
 *     <rpm>0</rpm>
 *     <debug>0</debug>
 *     -->
 *     
 *     <default_command_timeout_period>2</default_command_timeout_period> 
 *   </plugin>
 */
namespace gazebo
{

    void SevenEngineForce::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
	
      
      this->debug = false;
      if (_sdf->HasElement("debug"))
	this->debug = _sdf->Get<bool>("debug");
      
      //for debug purposes only.
      this->lastForce =  math::Vector3(0, 0, 0);
      this->lastTorque = math::Vector3(0, 0, 0);
      
      this->model = _parent;
      this->world = model->GetWorld();
      
      engine_forces = new double[7];
      
      resetEngineForces();
      
      this->isRPM = false;
      if (_sdf->HasElement("rpm"))
	this->isRPM = _sdf->Get<bool>("rpm");
      
      if (_sdf->HasElement("link"))
	this->link = this->model->GetLink(_sdf->Get<std::string>("link"));
      
      if (_sdf->HasElement("rx"))
	this->rx = _sdf->Get<double>("rx");
      if (_sdf->HasElement("ry"))
	this->ry = _sdf->Get<double>("ry");
      if (_sdf->HasElement("rz"))
	this->rz = _sdf->Get<double>("rz");
      if (_sdf->HasElement("rroll"))
	this->rroll = _sdf->Get<double>("rroll");
      
      
      this->forceCoef = 1;
      if (_sdf->HasElement("forceCoef"))
	this->forceCoef = _sdf->Get<double>("forceCoef");

      
      this->next_time_out = 0;
      this->default_command_timeout_period = -1;
      
      if (_sdf->HasElement("default_command_timeout_period"))
      {
	this->default_command_timeout_period = _sdf->Get<double>("default_command_timeout_period");
      }

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
	  boost::bind(&SevenEngineForce::OnUpdate, this, _1));
      
      
      // register ROS nodes
      rosnode_ = new ros::NodeHandle("gazebo/seven_engines_force");

      // initialize subscriber to water current
      ros::SubscribeOptions ops = ros::SubscribeOptions::create<seven_engines_force::engine_forces>(
		  model->GetName(), 1,
		  boost::bind(&SevenEngineForce::SetParamsCallBack, this, _1),
		  ros::VoidPtr(), &callback_queue_);
      body_force_subscriber_ = rosnode_->subscribe(ops);
      if (debug)
      {
	ROS_INFO("rx %f ry %f rz %f rroll %f forceCoef %f rpm %d link %s default_command_timeout_period %f", this->rx, this->ry, this->rz, this->rroll, this->forceCoef,
	    this->isRPM, this->link->GetName().c_str(), this->default_command_timeout_period);
      }

    }

    // Called by the world update start event
    void SevenEngineForce::OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      //call updates from ros message.
      callback_queue_.callAvailable();
      
      if (default_command_timeout_period == -1  ||  this->world->GetSimTime().Double() < this->next_time_out)
      {
	this->link->AddRelativeForce(getForce());
	this->link->AddRelativeTorque(getTorque());
      }
      else
      {
	this->next_time_out = 0;
	resetEngineForces();
      }      
    }

    math::Vector3 SevenEngineForce::getForce()
    {
      
      math::Vector3 force(0, 0, 0);
      if (isRPM)
	return force;
      else
      {
	force.x = this->forceCoef*(engine_forces[0] + engine_forces[1]);
	force.y = this->forceCoef*(engine_forces[2] + engine_forces[3]);
	force.z = this->forceCoef*(engine_forces[4] + engine_forces[5] + engine_forces[6]);
      }
      if (debug && this->lastForce != force)
      {
	ROS_INFO("force (%f, %f, %f)", force.x, force.y, force.z);
	this->lastForce = force;
      }
      return force;
    }
    
    math::Vector3 SevenEngineForce::getTorque()
    {
      
      math::Vector3 torque(0, 0, 0);
      if (isRPM)
	return torque;
      else
      {
	torque.x = this->rroll * (engine_forces[5] - engine_forces[6]);
	torque.y = this->rz * ( engine_forces[4] - (engine_forces[5] + engine_forces[6]) );
	torque.z = this->rx * (engine_forces[0] - engine_forces[1]) + this->ry*(engine_forces[2] - engine_forces[3]);
      }
      if (debug && this->lastTorque != torque)
      {
	ROS_INFO("torque (%f, %f, %f)", torque.x, torque.y, torque.z);
	this->lastTorque = torque;
      }
      return torque;
    }
    
    
    void SevenEngineForce::SetParamsCallBack (const seven_engines_force::engine_forcesConstPtr &_params)
    {
      
      for (int i = 0; i < 7; i++)
      {
	engine_forces[i]=_params->engine_forces[i];
      }
      
      if (default_command_timeout_period != -1)
      {
	this->next_time_out = this->world->GetSimTime().Double() + default_command_timeout_period;
      }
      
      if (debug)
      {
	ROS_INFO("set engine_forces to %f, %f, %f, %f, %f, %f, %f", engine_forces[0], engine_forces[1], engine_forces[2]
	  ,engine_forces[3], engine_forces[4], engine_forces[5], engine_forces[6]);
	if (default_command_timeout_period != -1)
	  ROS_INFO("next timeout at: %f", this->next_time_out);
      }
      
      
    }
    
    void SevenEngineForce::resetEngineForces()
    {
      for (int i = 0; i < 7 ; i++)
      {
	engine_forces[i]=0;
      }
      if (debug && (this->lastForce != math::Vector3(0,0,0) || this->lastTorque != math::Vector3(0,0,0)) )
	ROS_INFO("forces timed out. engines reset to 0.");
      this->lastForce = math::Vector3(0,0,0);
      this->lastTorque = math::Vector3(0,0,0);
    }
    

  };