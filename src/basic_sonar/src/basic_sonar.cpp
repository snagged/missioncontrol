#include <basic_sonar/basic_sonar.h>

#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/RaySensor.hh>

#include <boost/bind.hpp>

#include <vector>
#include <boost/thread/mutex.hpp>

namespace gazebo {

BasicSonar::BasicSonar()
{
}


BasicSonar::~BasicSonar()
{
}

void BasicSonar::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  sensor_ = boost::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
  if (!sensor_)
  {
  gzthrow("BasicSonar requires a Ray Sensor as its parent");
    return;
  }
  
  pixels_per_line = std::max(sensor_->GetRangeCount(), 1);
  
  gaussian_kernel = 0;
  if (_sdf->HasElement("gaussian_kernel"))
	gaussian_kernel = _sdf->Get<double>("gaussian_kernel");
  
  base_water_noise = 0;
  if (_sdf->HasElement("base_water_noise"))
	base_water_noise = _sdf->Get<double>("base_water_noise");
  
  default_scan_retro = 0;
  if (_sdf->HasElement("default_scan_retro"))
	default_scan_retro = _sdf->Get<double>("default_scan_retro"); 
  
  interpolation_limit = 0.5;
  if (_sdf->HasElement("interpolation_limit"))
	interpolation_limit = _sdf->Get<double>("interpolation_limit"); 

  if (_sdf->HasElement("service_name"))
	service_name = _sdf->Get<std::string>("service_name");
  
  if (service_name.empty())
  {
  gzthrow("BasicSonar requires a unique service name");
    return;
  }
  
  
  
  std::string worldName = sensor_->GetWorldName();
  world = physics::get_world(worldName);


  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
    << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  node_handle_ = new ros::NodeHandle("gazebo/basic_sonar/");
  service = node_handle_->advertiseService(service_name.c_str(), &BasicSonar::getSonarScan, this);

  sensor_->SetActive(true);
  
  ROS_INFO("loaded sonar plugin");
}


bool BasicSonar::getSonarScan(basic_sonar::sonar_serv::Request  &req,
             basic_sonar::sonar_serv::Response &res)
{
  std::vector<double>* result = get3DSonarScan();

  
  if (result->size() == 0){
    delete result;
    return false;
  }
  
  for (int i = 0; i < result->size(); i++){
    res.res.push_back(result->at(i));
  }
  delete result;
  return true;
}


std::vector<double>* BasicSonar::get3DSonarScan()
{
  sensor_->SetActive(false);
   
  int vertical_scans = std::max(sensor_->GetVerticalRangeCount(), 1);
  int horizontal_scans = std::max(sensor_->GetRangeCount(), 1);
  double min_range = sensor_->GetRangeMin();
  double max_range = sensor_->GetRangeMax();
  double resolution = (max_range - min_range) / pixels_per_line;
  int result_size = horizontal_scans*pixels_per_line;
  
  std::vector<double>* result = new std::vector<double>(result_size, 0);
  std::vector<double> noise_percentage(result_size, 0); // auxiliary vector used for calculating shadow.
  
  //Debug: print variables.
  //ROS_INFO("(vs,hs,mr,Mr,res,ppl): (%d,%d,%f,%f,%f,%d)", vertical_scans, horizontal_scans, min_range, max_range, resolution, pixels_per_line);
  
  createScanPictureFullInterpolation(horizontal_scans, vertical_scans, max_range, min_range, resolution, noise_percentage, result);
  createAccumulatedShadowAndNoise(horizontal_scans, vertical_scans, pixels_per_line, noise_percentage, result);
  sensor_->SetActive(true);
  return result; 
}


void BasicSonar::createScanPictureFullInterpolation(int horizontal_scans, int vertical_scans, double max_range, double min_range, double resolution, 
				   std::vector<double> &noise_percentage, std::vector<double>* result)
{
  std::vector<double> raw_scan;
  sensor_->GetRanges(raw_scan);
  for (int i = 0; i < horizontal_scans; i++)
  {
    double lastScanDist = 0, lastScanRetro = 0, currentScanDist = 0, currentScanRetro = 0;
    
    //initiate first values:
    changeRealScanValues(lastScanDist, lastScanRetro, horizontal_scans, i, max_range, 
			  raw_scan, 0, default_scan_retro);
    if (vertical_scans == 1) { //edge case: only one vertical scan, so linear interpolation isn't possible.
      currentScanDist = lastScanDist;
      currentScanRetro = lastScanRetro;
    } else {
      changeRealScanValues(currentScanDist, currentScanRetro, horizontal_scans, i, max_range, 
			       raw_scan, 1, default_scan_retro);
    }
    
    //loop through every vertical scan:
    for (int j = 0; j < vertical_scans; j++)
    {
      //update values:
      int vertical_array_position = j;
      lastScanDist = currentScanDist;
      lastScanRetro = currentScanRetro;
      if (vertical_array_position < vertical_scans - 1){  
	changeRealScanValues(currentScanDist, currentScanRetro, horizontal_scans, i, max_range, 
			      raw_scan, vertical_array_position + 1, default_scan_retro);
      }
      
      //update result scan:
      double relationBetweenFarAndCloseScans = std::max(currentScanDist,lastScanDist) / std::min(currentScanDist,lastScanDist);
      int position_in_line = (int) boost::math::round((lastScanDist - min_range) / resolution);
      
      updateResultAndNoise(result, noise_percentage, i, position_in_line, lastScanRetro);
      
       //if two scans are relatively close enough (but not the same)
      if (relationBetweenFarAndCloseScans <= (1 + interpolation_limit) && relationBetweenFarAndCloseScans != 1)
	interpolateRange(currentScanDist, lastScanDist, currentScanRetro, lastScanRetro, min_range, resolution, i, result);
    }
  }
}


void BasicSonar::interpolateRange(double currentScanDist, double lastScanDist, double currentScanRetro, double lastScanRetro, 
		      double min_range, double resolution, int line_index, std::vector<double>* result)
{
  int position_in_line = (int) boost::math::round((lastScanDist - min_range) / resolution);
  int next_position_in_line = (int) boost::math::round((currentScanDist - min_range) / resolution);
  
  int closer_position = (int) std::min(next_position_in_line, position_in_line);
  int farther_position = (int) std::max(next_position_in_line, position_in_line);
  
  double closer_retro = (closer_position == position_in_line) ? lastScanRetro : currentScanRetro;
  double farther_retro = (farther_position == next_position_in_line) ? currentScanRetro : lastScanRetro;
  
  for (int k = closer_position+1; k < farther_position; k++)
  {
    result->at((pixels_per_line * line_index) + k - 1 ) = ( (k - closer_position)*farther_retro + (farther_position - k)*closer_retro ) / (farther_position - closer_position);
  }  
}


void BasicSonar::changeRealScanValues(double &range, double &retro, int horizontal_scans, int line_index, double max_range,
				      std::vector<double> &raw_scan, double vertical_position, double default_value)
{
  range = raw_scan[line_index + (vertical_position)*horizontal_scans];
  retro = sensor_->GetRetro(line_index + (vertical_position)*horizontal_scans);
  if (range < max_range && retro == 0)
    retro = default_value;
}


void BasicSonar::updateResultAndNoise(std::vector<double>* result_vec, std::vector<double> &noise_percentage_vec, 
				      int line_index, int position_in_line, double scanVal)
{
	//mark that the ray pixel isn't supposed to be shaddowed.
  	noise_percentage_vec.at((pixels_per_line * line_index) + position_in_line -1)++;
      	
	result_vec->at((pixels_per_line * line_index) + position_in_line -1 ) = std::max(result_vec->at((pixels_per_line * line_index) 
	  + position_in_line -1 ), scanVal);
}


void BasicSonar::createAccumulatedShadowAndNoise(int horizontal_scans, int vertical_scans, int pixels_per_line, 
					 std::vector<double> noise_percentage, std::vector<double>* result)
{
  for (int i=0; i < horizontal_scans; i++)
  {
    int accumulated_num_of_rays = 0;
    for (int j=0; j < pixels_per_line; j++)
    {
      if (noise_percentage.at(i*pixels_per_line + j) > 0)
      {
	accumulated_num_of_rays += noise_percentage.at(i*pixels_per_line + j);
	result->at(i*pixels_per_line + j) += gaussianNoise(0,gaussian_kernel);
      }
      else if (result->at(i*pixels_per_line + j) == 0)
      {
	result->at(i*pixels_per_line + j) = ( 1 - (accumulated_num_of_rays/ (double) vertical_scans)) * base_water_noise + gaussianNoise(0,gaussian_kernel);
      }
      else
      {
	result->at(i*pixels_per_line + j) += gaussianNoise(0,gaussian_kernel);
      }
    }
  }
}


double BasicSonar::gaussianNoise(double mu, double sigma)
{
  // using Box-Muller transform to generate two independent standard normally disbributed normal variables
  double U = (double)rand()/(double) RAND_MAX; 
  double V = (double)rand()/(double) RAND_MAX; 
  double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
  
  // we'll just use X scale to our mu and sigma
  X = sigma *X + mu;
  return X;
}


void BasicSonar::Update()
{

}
  
  
  GZ_REGISTER_SENSOR_PLUGIN(BasicSonar)
}//namespace gazebo