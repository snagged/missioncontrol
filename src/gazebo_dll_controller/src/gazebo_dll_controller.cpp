#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <gazebo/gazebo.hh>

#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <tinyxml.h>
#include <urdf_parser/urdf_parser.h>
#include <gazebo/math/Pose.hh>


#include <gazebo_dll_controller/gazebo_dll_controller.h>

using std::cout;
using std::endl;
using std::string;

namespace gazebo
{


void GazeboDllControllerPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
    this->world_ = _world;

    /*if(_sdf->HasElement("bluefin"))  
      description_ = _sdf->Get<std::string>("descriptionParam");*/

    // Register plugin update
    update_event_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboDllControllerPlugin::Update, this));
}


void GazeboDllControllerPlugin::Update()
{
}

}
