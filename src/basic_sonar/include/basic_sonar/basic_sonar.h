#ifndef BASIC_SONAR
#define BASIC_SONAR

#include <gazebo/common/Plugin.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include "basic_sonar/sonar_serv.h"
#include "basic_sonar/sonar_image_serv.h"
#include <vector>


namespace gazebo
{
  
class BasicSonar : public SensorPlugin
{
  public:
    BasicSonar();
    virtual ~BasicSonar();
    
    physics::WorldPtr world;
    sensors::RaySensorPtr sensor_;
    
    int pixels_per_line;
    double gaussian_kernel;
    double base_water_noise;
    double default_scan_retro;
    double interpolation_limit;
    std::string service_name;
    
    ///\brief generates a ros response containing a sonar scan array.
    bool getSonarScan(basic_sonar::sonar_serv::Request &req,
             basic_sonar::sonar_serv::Response &res);
    
    ///\brief returns a random value according to a Normal distribution.
    static double gaussianNoise(double mu, double sigma);
  
  
  protected:
    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
    virtual void Update();

    
    ///\brief generates a vector containing a sonar scan array.
    std::vector<double>* get3DSonarScan();
   
      
    
    ///\brief updates result vector with the scan value, and updates the noise_percentage_vec that the position was given.
    void updateResultAndNoise(std::vector<double>* result_vec, std::vector<double> &noise_percentage_vec,
				      int line_index, int position_in_line, double scanVal);
    
    
    ///\brief updates the given range and retro doubles to the current value.
    void changeRealScanValues(double &range, double &retro, int horizontal_scans, int line_index, double max_range,
				      std::vector<double> &raw_scan, double vertical_position, double default_value);

    
    ///\brief another version for the main function for get3DSonarScan(). 
    ///	for every real ray, it calculates the pixel in the picture 
    ///	according to range, and gives it the retro value.
    ///	between every two rays, if they are close enough, it interpolates the result to contain retro for pixels in between.
    void createScanPictureFullInterpolation(int horizontal_scans, int vertical_scans, double max_range, double min_range, double resolution, 
				   std::vector<double> &noise_percentage, std::vector<double>* result);
    

    ///\brief auxilary function - calculates the pixels from last scan to current scan, and colors them with a linear interpoation
    ///	of the last scan retro and the current one.
    void interpolateRange(double currentScanDist, double lastScanDist, double currentScanRetro, double lastScanRetro, 
		      double min_range, double resolution, int line_index, std::vector<double>* result);
    
    
    ///\brief creates a shdow behind the scanned points, according to noise percentage.
    ///	the noise_percentage should contain number of rays that hit every pixel. 
    ///	if the number is 0, then a noise is generated according to base_water_noise, and number of rays that hit before.
    ///	if the number is larger. simply adds noise.
    void createAccumulatedShadowAndNoise(int horizontal_scans, int vertical_scans, int pixels_per_line, 
				     std::vector<double> noise_percentage, std::vector<double>* result);
    
    

    
    ros::NodeHandle* node_handle_;

    event::ConnectionPtr updateConnection;
    ros::ServiceServer service;
    
};
} // namespace gazebo
#endif // BASIC_SONAR