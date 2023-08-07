#ifndef ALS_ROS_STATIC_TRANSFORMS_H
#define ALS_ROS_STATIC_TRANSFORMS_H

#include <tf/transform_datatypes.h>
// adapted from scan_tools/laser_scan_matcher/src/static_transforms.cpp
namespace als_ros {
/**
 * @brief Get the transform from base_scan to base_footprint
 * base_footprint frame, forward, left, up
 * base_link frame, forward, left, up
 * base_scan_frame, forward, left, up
 * laser_frame when mounted on the platform, backward, right, up
*/
inline tf::Transform get_base_scan_T_laser(std::string model) {
  if (model == "turtlebot3_burger") {
    tf::Vector3 translation(0.0, 0.0, 0.0);
    tf::Matrix3x3 rot(-1, 0, 0, 
                      0, -1, 0, 
                      0, 0, 1);
    return tf::Transform(rot, translation);
  } else {
    return tf::Transform::getIdentity();
  }
}

inline tf::Transform get_base_link_T_base_footprint(std::string model) {
  return tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(0.0, 0.0, -0.1));
}

/**
 * @brief Get the transform from base_footprint to base_scan
 * rosrun tf tf_echo base_footprint base_scan
*/
inline tf::Transform get_base_footprint_T_base_scan(std::string model) {
  tf::Transform base_footprint_T_base_scan;
  if (model == "turtlebot3_burger") {
    tf::Vector3 translation(-0.032, 0.000, 0.182);
    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, 0.0);
    base_footprint_T_base_scan = tf::Transform(quat, translation);
  } else {
    std::cerr << "Unknown model: " << model << std::endl;
    base_footprint_T_base_scan.setIdentity();
  }
  return base_footprint_T_base_scan;
}

inline tf::Transform get_base_link_T_base_scan(std::string model) {
  tf::Transform base_link_T_base_scan;
  if (model == "turtlebot3_burger") {
    tf::Vector3 translation(-0.032, 0.000, 0.172);
    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, 0.0);
    base_link_T_base_scan = tf::Transform(quat, translation);
  } else {
    std::cerr << "Unknown model: " << model << std::endl;
    base_link_T_base_scan.setIdentity();
  }
  return base_link_T_base_scan;
}

inline tf::Transform get_base_link_T_laser(std::string model) {
  return get_base_link_T_base_scan(model) * get_base_scan_T_laser(model);
}

}  // namespace als_ros
#endif