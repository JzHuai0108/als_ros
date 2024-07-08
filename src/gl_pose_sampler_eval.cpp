/****************************************************************************
 * als_ros: An Advanced Localization System for ROS use with 2D LiDAR
 * Copyright (C) 2022 Naoki Akai
 *
 * Licensed under the Apache License, Version 2.0 (the “License”);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an “AS IS” BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @author Naoki Akai
 ****************************************************************************/

#include <fstream>
#include <vector>

#include <ros/ros.h>
#include <octomap_server/OctomapServer.h>

#include <als_ros/GLPoseSampler.h>
#include <als_ros/pointcloud_to_laserscan.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

struct TimedPose {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ros::Time t;
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    
    TimedPose(const ros::Time &_t, const Eigen::Vector3d &_p, const Eigen::Quaterniond &_q) : t(_t), p(_p), q(_q) {}
};

typedef std::vector<TimedPose, Eigen::aligned_allocator<TimedPose>> TimedPoseVector;

size_t loadTumOdometry(const std::string &filename, TimedPoseVector &poses) {
    std::ifstream ifs(filename);
    if (!ifs) {
        ROS_ERROR("Failed to open file: %s", filename.c_str());
        return 0;
    }
    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream iss(line);
        std::string timestr;
        std::getline(iss, timestr, ' ');
        Eigen::Vector3d p;
        Eigen::Quaterniond q;
        iss >> p.x() >> p.y() >> p.z() >> q.x() >> q.y() >> q.z() >> q.w();
        size_t dotpos = timestr.find('.');
        int32_t sec = std::stol(timestr.substr(0, dotpos));
        int32_t nsec;
        int nseclen = timestr.size() - dotpos - 1;
        int64_t nsecx = std::stol(timestr.substr(dotpos + 1));
        if (nseclen <= 9) {
            nsecx *= std::pow(10, 9 - nseclen);
            nsec = (int)nsecx;
        } else {
            double fraction = nsecx / std::pow(10, nseclen - 9);
            nsec = std::round(fraction);
        }

        ros::Time time(sec, nsec);
        poses.emplace_back(time, p, q);
    }
    // print size and first and last pose
    if (!poses.empty()) {
        ROS_INFO("Loaded %lu poses", poses.size());
        ROS_INFO("First pose: %d.%09d, %.6f, %.6f, %.6f, %.9f, %.9f, %.9f, %.9f",
                 poses.front().t.sec, poses.front().t.nsec, poses.front().p.x(), poses.front().p.y(), poses.front().p.z(),
                 poses.front().q.x(), poses.front().q.y(), poses.front().q.z(), poses.front().q.w());
        ROS_INFO("Last pose: %d.%09d, %.6f, %.6f, %.6f, %.9f, %.9f, %.9f, %.9f",
                poses.back().t.sec, poses.back().t.nsec, poses.back().p.x(), poses.back().p.y(), poses.back().p.z(),
                poses.back().q.x(), poses.back().q.y(), poses.back().q.z(), poses.back().q.w());
    }
    return poses.size();
}

void recordPosesCB(const geometry_msgs::PoseArray &poses, const ros::Time &time, const std::string &outfile) {
    ROS_INFO("Received %lu poses", poses.poses.size());
    std::ofstream ofs(outfile, std::ios::app);
    if (!ofs) {
        ROS_ERROR("Failed to open file: gl_sampled_poses.txt");
        return;
    }
    for (size_t i = 0; i < poses.poses.size(); i++) {
        ofs << time.sec << "." << std::setw(9) << std::setfill('0') << time.nsec << " "
            << std::fixed << std::setprecision(8)
            << poses.poses[i].position.x << " " << poses.poses[i].position.y << " "
            << poses.poses[i].position.z << " " << std::setprecision(9)
            << poses.poses[i].orientation.x << " " << poses.poses[i].orientation.y << " " 
            << poses.poses[i].orientation.z << " " << poses.poses[i].orientation.w << std::endl;
    }
    ofs.close();
}

/**
 * @brief Map generation node.
 * copied from https://github.com/strawlab/navigation/blob/master/map_server/src/map_saver.cpp
 */
class MapGenerator 
{

  public:
    MapGenerator(const std::string &outdir, const std::string& mapname) : 
        outdir_(outdir), mapname_(mapname), saved_map_(false)
    {
    //   ros::NodeHandle n;
    //   ROS_INFO("Waiting for the map");
    //   map_sub_ = n.subscribe("map", 1, &MapGenerator::mapCallback, this);
    }

    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
    {
      ROS_INFO("Received a %d X %d map @ %.3f m/pix",
               map->info.width,
               map->info.height,
               map->info.resolution);

      std::string mapdatafile = outdir_ + '/' + mapname_ + ".pgm";
      ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
      FILE* out = fopen(mapdatafile.c_str(), "w");
      if (!out)
      {
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        return;
      }

      fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
              map->info.resolution, map->info.width, map->info.height);
      for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {
          unsigned int i = x + (map->info.height - y - 1) * map->info.width;
          if (map->data[i] == 0) { //occ [0,0.1)
            fputc(254, out);
          } else if (map->data[i] == +100) { //occ (0.65,1]
            fputc(000, out);
          } else { //occ [0.1,0.65]
            fputc(205, out);
          }
        }
      }

      fclose(out);

      std::string mapmetadatafile = outdir_ + '/' + mapname_ + ".yaml";
      ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
      FILE* yaml = fopen(mapmetadatafile.c_str(), "w");


      /*
resolution: 0.100000
origin: [0.000000, 0.000000, 0.000000]
#
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196

       */

      geometry_msgs::Quaternion orientation = map->info.origin.orientation;
      tf::Quaternion q(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

    //   btMatrix3x3 mat(btQuaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    //   double yaw, pitch, roll;
    //   mat.getEulerYPR(yaw, pitch, roll);

      fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
              mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

      fclose(yaml);

      ROS_INFO("Done saving map to %s and %s", mapdatafile.c_str(), mapmetadatafile.c_str());
      saved_map_ = true;
    }

    std::string outdir_;
    std::string mapname_;
    // ros::Subscriber map_sub_;
    bool saved_map_;

};

class MyOctomapServer : public octomap_server::OctomapServer
{
public:
    MyOctomapServer() : octomap_server::OctomapServer() {}
    virtual ~MyOctomapServer() {}
    using octomap_server::OctomapServer::m_filterSpeckles;
    using octomap_server::OctomapServer::m_maxRange;
    using octomap_server::OctomapServer::m_minRange;
    using octomap_server::OctomapServer::m_res;
    using octomap_server::OctomapServer::m_pointcloudMinZ;
    using octomap_server::OctomapServer::m_pointcloudMaxZ;
    using octomap_server::OctomapServer::m_worldFrameId;
    using octomap_server::OctomapServer::m_latchedTopics;
    using octomap_server::OctomapServer::m_gridmap;
};

void transformTumOdometry(TimedPoseVector &poses, const Eigen::Vector3d &trans) {
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.translation() = Eigen::Vector3d(trans[0], trans[1], 0);
    Eigen::AngleAxisd aa(trans[2], Eigen::Vector3d::UnitZ());
    T.linear() = aa.toRotationMatrix();
    for (size_t i = 0; i < poses.size(); i++) {
        Eigen::Vector3d p = poses[i].p;
        Eigen::Quaterniond q = poses[i].q;
        Eigen::Affine3d pose = Eigen::Affine3d::Identity();
        pose.translation() = p;
        pose.linear() = q.toRotationMatrix();
        Eigen::Affine3d newpose = T * pose;
        poses[i].p = newpose.translation();
        poses[i].q = Eigen::Quaterniond(newpose.rotation());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gl_pose_sampler");
    als_ros::GLPoseSampler sampler; // This will wait for the occupancy grid map message.
    ros::NodeHandle &nh = sampler.getNodeHandle();
    std::string posefile;
    nh.param("odometry_file", posefile, std::string(""));
    std::string outdir;
    nh.param("output_dir", outdir, std::string(""));
    double mock_map_T_odom_x = 0;
    double mock_map_T_odom_y = 0;
    double mock_map_T_odom_theta = 0 * M_PI / 180;
    nh.param("/mock_map_T_odom_x", mock_map_T_odom_x, 0.0);
    nh.param("/mock_map_T_odom_y", mock_map_T_odom_y, 0.0);
    nh.param("/mock_map_T_odom_theta_deg", mock_map_T_odom_theta, 0.0);
    ROS_INFO("mock_map_T_odom_x: %f, mock_map_T_odom_y: %f, mock_map_T_odom_theta_deg: %f", mock_map_T_odom_x, mock_map_T_odom_y, mock_map_T_odom_theta);
    mock_map_T_odom_theta *= M_PI / 180;
    double playrate = 1;
    nh.param("/playrate", playrate, 1.0);

    PointCloudToLaserScanNode pc2scan(nh);
    nav_msgs::OccupancyGridConstPtr submap_;

    TimedPoseVector tum_poses;
    if (loadTumOdometry(posefile, tum_poses) == 0) {
        ROS_ERROR("Failed to load TUM odometry file: %s", posefile.c_str());
        return 1;
    }
    // we transform the original odometry to make the problem more realistic.
    transformTumOdometry(tum_poses, {mock_map_T_odom_x, mock_map_T_odom_y, mock_map_T_odom_theta});

    // check that files of pose time as names exist
    std::string posedir;
    size_t slashpos = posefile.find_last_of('/');
    if (slashpos != std::string::npos) {
        posedir = posefile.substr(0, slashpos + 1);
    }
    int badfile = 0;
    for (size_t i = 0; i < tum_poses.size(); i++) {
        std::stringstream ss;
        ss << "scan_" << tum_poses[i].t.sec << std::setw(9) << std::setfill('0') << tum_poses[i].t.nsec << ".pcd";
        std::string filename = posedir + ss.str();
        std::ifstream ifs(filename);
        if (!ifs) {
            ROS_ERROR("Failed to open file: %s", filename.c_str());
            badfile++;
        }
        ifs.close();
    }
    if (badfile > 0) {
        ROS_ERROR("Failed to open %d files", badfile);
        return 1;
    } else {
        ROS_INFO("All pcd files exist");
    }
    std::string outfile = outdir + "/gl_sampled_poses.txt";
    std::ofstream ofs(outfile, std::ios::trunc);
    ofs.close();

    tf::TransformBroadcaster tf_broadcaster;

    MyOctomapServer map_server;
    map_server.m_filterSpeckles = true;
    map_server.m_maxRange = 40.0;
    map_server.m_pointcloudMaxZ = 1.0;
    map_server.m_pointcloudMinZ = -0.5;
    map_server.m_res = 0.05;
    map_server.m_worldFrameId = "map";
    map_server.m_latchedTopics = true;

    ros::Rate rate(playrate);
    int halfwin = 2; // This has to be no more than half of the octomap server filter size 5 to avoid tf extrapolation errors.
    int lastpublishedtfid = 0;
    tf::StampedTransform W_T_Bzup;
    int k = lastpublishedtfid;
    W_T_Bzup.setOrigin(tf::Vector3(tum_poses[k].p.x(), tum_poses[k].p.y(), tum_poses[k].p.z()));
    W_T_Bzup.setRotation(tf::Quaternion(tum_poses[k].q.x(), tum_poses[k].q.y(), tum_poses[k].q.z(), tum_poses[k].q.w()));
    W_T_Bzup.frame_id_ = sampler.odomFrame_;
    W_T_Bzup.child_frame_id_ = sampler.baseLinkFrame_;
    W_T_Bzup.stamp_ = tum_poses[k].t;
    tf_broadcaster.sendTransform(W_T_Bzup);

    // We do not know map_T_odom, but we publish a mock value for visualization.
    tf::StampedTransform map_T_odom;
    map_T_odom.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    map_T_odom.setRotation(q);
    map_T_odom.frame_id_ = sampler.mapFrame_;
    map_T_odom.child_frame_id_ = sampler.odomFrame_;
    map_T_odom.stamp_ = tum_poses[k].t;
    tf_broadcaster.sendTransform(map_T_odom);

    for (int i = 0; i < (int)tum_poses.size(); i++) {
        int j = 0;
        int left = i < halfwin ? 0 : i - halfwin;
        int right = i + halfwin >= tum_poses.size() ? tum_poses.size() - 1 : i + halfwin;
        // 1. publish pont clouds and tf poses to octomap server to build a submap
        // and get the submap in the end.
        std::vector<sensor_msgs::LaserScan> scans2d;
        for (j = left; j <= right; j++) {
            std::stringstream ss;
            ss << "scan_" << tum_poses[j].t.sec << std::setw(9) << std::setfill('0') << tum_poses[j].t.nsec << ".pcd";
            std::string filename = posedir + ss.str();
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
            if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(filename, *cloud) == -1) {
                ROS_ERROR("Failed to load pcd file: %s", filename.c_str());
                return 1;
            }
            int k = j + 1;
            if (k > lastpublishedtfid && k < tum_poses.size()) {
                // publish the pose as tf to octomap
                tf::StampedTransform W_T_Bzup;
                W_T_Bzup.setOrigin(tf::Vector3(tum_poses[k].p.x(), tum_poses[k].p.y(), tum_poses[k].p.z()));
                W_T_Bzup.setRotation(tf::Quaternion(tum_poses[k].q.x(), tum_poses[k].q.y(), tum_poses[k].q.z(), tum_poses[k].q.w()));
                W_T_Bzup.frame_id_ = sampler.odomFrame_;
                W_T_Bzup.child_frame_id_ = sampler.baseLinkFrame_;
                W_T_Bzup.stamp_ = tum_poses[k].t;
                tf_broadcaster.sendTransform(W_T_Bzup);
                map_T_odom.stamp_ = tum_poses[k].t;
                tf_broadcaster.sendTransform(map_T_odom);
                lastpublishedtfid = k;
            }
            ros::spinOnce();

            // publish cloud on octomap cloud_in
            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(*cloud, cloud_msg);
            cloud_msg.header.stamp = tum_poses[j].t;
            cloud_msg.header.frame_id = sampler.baseLinkFrame_;
            sensor_msgs::PointCloud2ConstPtr cloud_msg_ptr = boost::make_shared<sensor_msgs::PointCloud2>(cloud_msg);
            map_server.insertCloudCallback(cloud_msg_ptr);

            ros::spinOnce();
            if (j == right) {
                nav_msgs::OccupancyGridPtr submap = boost::make_shared<nav_msgs::OccupancyGrid>(map_server.m_gridmap);
                submap->header.stamp = tum_poses[right].t;
                submap->header.frame_id = sampler.odomFrame_;
                submap->info.origin.orientation.w = 1.0;
                submap_ = submap;
                std_srvs::Empty empty;
                map_server.resetSrv(empty.request, empty.response);

                sensor_msgs::LaserScan scan;
                pc2scan.cloudCallback(cloud_msg_ptr);
                scans2d.push_back(*pc2scan.scan_msg_);
            }
        }

        // 2. with the submap and the odometry pose, call pose sampler to get the candidate localizations.
        nav_msgs::Odometry odom;
        odom.header.stamp = tum_poses[right].t;
        odom.header.frame_id = sampler.odomFrame_;
        odom.child_frame_id = sampler.baseLinkFrame_;
        odom.pose.pose.position.x = tum_poses[right].p.x();
        odom.pose.pose.position.y = tum_poses[right].p.y();
        odom.pose.pose.position.z = tum_poses[right].p.z();
        odom.pose.pose.orientation.x = tum_poses[right].q.x();
        odom.pose.pose.orientation.y = tum_poses[right].q.y();
        odom.pose.pose.orientation.z = tum_poses[right].q.z();
        odom.pose.pose.orientation.w = tum_poses[right].q.w();
        sampler.odomCB(boost::make_shared<nav_msgs::Odometry>(odom));
        sampler.setKeyScans(scans2d);
        sampler.submapCB(submap_);
        // Save poses for evaluation.
        recordPosesCB(sampler.poses_, tum_poses[right].t, outfile);
        // Save submaps for debug.
        // std::string mapname = "submap_" + std::to_string(right);
        // MapGenerator map_generator(outdir, mapname);
        // map_generator.mapCallback(submap_);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
