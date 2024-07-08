

#include <ros/node_handle.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

/**
* Class to process incoming pointclouds into laserscans.
* Some initial code was pulled from the defunct turtlebot pointcloud_to_laserscan implementation.
*/
class PointCloudToLaserScanNode
{
public:
  explicit PointCloudToLaserScanNode(ros::NodeHandle &nh);

  virtual ~PointCloudToLaserScanNode();

  void cloudCallback(sensor_msgs::PointCloud2::ConstPtr cloud_msg);

  std::unique_ptr<sensor_msgs::LaserScan> scan_msg_;
private:
  // ROS Parameters
  std::string target_frame_;
  double tolerance_;
  double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_,
    range_max_;
  bool use_inf_;
  double inf_epsilon_;
};
