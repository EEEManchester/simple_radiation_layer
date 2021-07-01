#ifndef RAD_LAYER_H_
#define RAD_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_layer.h>
#include <nav_msgs/OccupancyGrid.h>
//#include <costmap_2d/GenericPluginConfig.h>
#include <radiation_layer/RadiationLayerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

//#include <message_filters/subscriber.h>
//#include <sensor_msgs/Image.h>
#include <gazebo_radiation_plugins/Simulated_Radiation_Msg.h>
//#include <tf2_ros/buffer.h>
//#include <ros/time.h>

namespace radiation_layer_namespace
{

class RadLayer : public costmap_2d::CostmapLayer //costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  RadLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();


private:
  //void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  // dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  void reconfigureCB(radiation_layer::RadiationLayerConfig &config, uint32_t level);
  dynamic_reconfigure::Server<radiation_layer::RadiationLayerConfig> *dsrv_;

  void radiationCB(const gazebo_radiation_plugins::Simulated_Radiation_Msg& rad_msg); // Callback for incoming radiation messages
  void updateObservations(std::list<std::pair<unsigned int, float> > &updates); // Process recent observations and return value to be added to costmap
  void publishLayer(std::string);
  
  float* averages_;
  unsigned int* n_obs_;
  unsigned int averages_size_;

  bool update_full_layer_;
  bool rolling_window_;

  void getCache(std::list<std::pair<std::pair<double,double>, std::pair<int,float> > > &observation_cache);

  double upper_threshold_, lower_threshold_, ut_, lt_;
  int upper_threshold_scale_, lower_threshold_scale_;
  unsigned int scaledValue(float value);

  bool inflate_radiation_;
  int combination_method_;
  std::vector<geometry_msgs::Point> sensor_footprint_;

  ros::Subscriber radiation_sub_;
  ros::Publisher layer_pub_;
  std::list<gazebo_radiation_plugins::Simulated_Radiation_Msg> radiation_msg_buffer_;
  std::string global_frame_; // Global frame of costmap

  boost::recursive_mutex lock_; // Stop observation buffer being modified by multiple functions at once
  boost::recursive_mutex cache_;  // Stop cache from being modified when performing resizing

  std::vector<geometry_msgs::Point> makeSensorFootprintFromParams(ros::NodeHandle& nh);

};
}
#endif
