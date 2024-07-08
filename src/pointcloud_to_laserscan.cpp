#include <als_ros/pointcloud_to_laserscan.h>

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <ros/console.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

PointCloudToLaserScanNode::PointCloudToLaserScanNode(ros::NodeHandle &nh) {
    nh.param("/target_frame", target_frame_, std::string("Bzup"));
    nh.param("/transform_tolerance", tolerance_, 0.01);
    nh.param("/min_height", min_height_, std::numeric_limits<double>::min());
    nh.param("/max_height", max_height_, std::numeric_limits<double>::max());
    nh.param("/angle_min", angle_min_, -M_PI);
    nh.param("/angle_max", angle_max_, M_PI);
    nh.param("/angle_increment", angle_increment_, M_PI / 180.0);
    nh.param("/scan_time", scan_time_, 1.0 / 30.0);
    nh.param("/range_min", range_min_, 0.0);
    nh.param("/range_max", range_max_, std::numeric_limits<double>::max());
    nh.param("/inf_epsilon", inf_epsilon_, 1.0);
    nh.param("/use_inf", use_inf_, true);
    ROS_INFO_STREAM("Target frame: " << target_frame_ << ", min height " << min_height_
                     << ", max height " << max_height_ << ", angle min " << angle_min_ 
                     << ", angle max " << angle_max_ << ", angle increment " 
                     << angle_increment_ << ", scan time " << scan_time_ << ", range min "
                     << range_min_ << ", range max " << range_max_ << ", inf epsilon "
                     << inf_epsilon_ << ", use inf " << use_inf_);
}

PointCloudToLaserScanNode::~PointCloudToLaserScanNode() {
}

void PointCloudToLaserScanNode::cloudCallback(
  sensor_msgs::PointCloud2::ConstPtr cloud_msg)
{
  // build laserscan output
  auto scan_msg = std::make_unique<sensor_msgs::LaserScan>();
  scan_msg->header = cloud_msg->header;
  if (!target_frame_.empty()) {
    scan_msg->header.frame_id = target_frame_;
  }

  scan_msg->angle_min = angle_min_;
  scan_msg->angle_max = angle_max_;
  scan_msg->angle_increment = angle_increment_;
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_) {
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
  }

  // Transform cloud if necessary
  if (scan_msg->header.frame_id != cloud_msg->header.frame_id) {
    ROS_ERROR_STREAM("Transforming point cloud from " << cloud_msg->header.frame_id << " to " <<
      scan_msg->header.frame_id << " is not implemented yet.");
    return;
  }

  // Iterate through pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
    iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n",
        *iter_x, *iter_y, *iter_z);
      continue;
    }

    if (*iter_z > max_height_ || *iter_z < min_height_) {
      ROS_DEBUG("rejected for height %f not in range (%f, %f)\n",
        *iter_z, min_height_, max_height_);
      continue;
    }

    double range = hypot(*iter_x, *iter_y);
    if (range < range_min_) {
      ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
        range, range_min_, *iter_x, *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_) {
      ROS_DEBUG("rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
        range, range_max_, *iter_x, *iter_y, *iter_z);
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
      ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n",
        angle, scan_msg->angle_min, scan_msg->angle_max);
      continue;
    }

    // overwrite range at laserscan ray if new range is smaller
    int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
    if (range < scan_msg->ranges[index]) {
      scan_msg->ranges[index] = range;
    }
  }
  scan_msg_ = std::move(scan_msg);
}
