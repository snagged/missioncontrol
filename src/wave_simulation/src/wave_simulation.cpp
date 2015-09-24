#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <ros/ros.h>
#include <wave_simulation/wave_simulation.h>


namespace gazebo
{

    void WaveSimulation::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
	
      
      this->debug = false;
      if (_sdf->HasElement("debug"))
	this->debug = _sdf->Get<bool>("debug");
      
      this->model = _parent;
      this->world = model->GetWorld();
      
      /*if (_sdf->HasElement("rpm"))
	this->isRPM = _sdf->Get<bool>("rpm");*/

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
	  boost::bind(&WaveSimulation::OnUpdate, this, _1));

      if (debug)
      {
	ROS_INFO("debug message");
      }

    }

    // Called by the world update start event
    void WaveSimulation::OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      
    }

  
};