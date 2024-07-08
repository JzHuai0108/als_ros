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
 * To implement this program, a following paper was referred.
 * https://arxiv.org/pdf/1908.01863.pdf
 *
 * @author Naoki Akai
 ****************************************************************************/

#ifndef __GL_POSE_SAMPLER_H__
#define __GL_POSE_SAMPLER_H__

#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
// #include <opencv2/opencv.hpp>
#include <als_ros/Pose.h>

namespace als_ros {

class Keypoint {
private:
    int u_, v_;
    double x_, y_;
    char type_;
    // type values -2, -1, 0, or 1.
    // -2: invalid, -1: local minima, 0: suddle, 1: local maxima

public:
    Keypoint(void):
        u_(0), v_(0), x_(0.0), y_(0.0), type_(-2) {}

    Keypoint(int u, int v):
        u_(u), v_(v), x_(0.0), y_(0.0), type_(-2) {}

    Keypoint(double x, double y):
        u_(0), v_(0), x_(x), y_(y), type_(-2) {}

    Keypoint(int u, int v, double x, double y):
        u_(u), v_(v), x_(x), y_(y), type_(-2) {}

    Keypoint(int u, int v, double x, double y, char type):
        u_(u), v_(v), x_(x), y_(y), type_(type) {}

    inline int getU(void) { return u_; }
    inline int getV(void) { return v_; }
    inline double getX(void) { return x_; }
    inline double getY(void) { return y_; }
    inline char getType(void) { return type_; }

    inline void setU(int u) { u_ = u; }
    inline void setV(int v) { v_ = v; }
    inline void setX(double x) { x_ = x; }
    inline void setY(double y) { y_ = y; }
    inline void setType(char type) { type_ = type; }
}; // class Keypoint

class SDFOrientationFeature {
private:
    double dominantOrientation_;
    double averageSDF_;
    std::vector<int> relativeOrientationHist_;

public:
    SDFOrientationFeature(void) {}

    SDFOrientationFeature(double dominantOrientation, double averageSDF, std::vector<int> relativeOrientationHist):
        dominantOrientation_(dominantOrientation),
        averageSDF_(averageSDF),
        relativeOrientationHist_(relativeOrientationHist) {}

    inline double getDominantOrientation(void) { return dominantOrientation_; }
    inline double getAverageSDF(void) { return averageSDF_; }
    std::vector<int> getRelativeOrientationHist(void) { return relativeOrientationHist_; }
    int getRelativeOrientationHist(int idx) { return relativeOrientationHist_[idx]; }
}; // class SDFOrientationFeature

class GLPoseSampler {
private:
    ros::NodeHandle nh_;

    std::string mapName_, scanName_, odomName_, posesName_, localMapName_, sdfKeypointsName_, localSDFKeypointsName_;
    ros::Subscriber mapSub_, scanSub_, odomSub_;
    ros::Publisher posesPub_, localMapPub_, sdfKeypointsPub_, localSDFKeypointsPub_;

    Pose baseLink2Laser_;

    int mapWidth_, mapHeight_;
    double mapResolution_;
    Pose mapOrigin_;
    std::vector<signed char> mapData_;
    bool gotMap_;

    sensor_msgs::LaserScan scan_;
    double keyScanIntervalDist_, keyScanIntervalYaw_;
    std::vector<sensor_msgs::LaserScan> keyScans_;
    int keyScansNum_;

    Pose odomPose_;
    std::vector<Pose> keyPoses_;
    bool gotOdom_;

    std::vector<Keypoint> sdfKeypoints_;
    std::vector<SDFOrientationFeature> sdfOrientationFeatures_;
    visualization_msgs::Marker sdfKeypointsMarker_;

    double gradientSquareTH_;
    double keypointsMinDistFromMap_;
    double sdfFeatureWindowSize_;
    double averageSDFDeltaTH_;
    bool addRandomSamples_, addOppositeSamples_;
    int randomSamplesNum_;
    double positionalRandomNoise_, angularRandomNoise_, matchingRateTH_;

    tf::TransformBroadcaster tfBroadcaster_;
    tf::TransformListener tfListener_;

public:
    GLPoseSampler(void);

    void spin(void) {
        ros::spin();
    }

    ros::NodeHandle &getNodeHandle(void) {
        return nh_;
    }

    void submapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    void odomCB(const nav_msgs::Odometry::ConstPtr &msg);

    void setKeyScans(std::vector<sensor_msgs::LaserScan> &keyScans) {
        keyScans_ = keyScans;
        keyScansNum_ = keyScans.size();
    }

    geometry_msgs::PoseArray poses_;
    std::string mapFrame_, odomFrame_, baseLinkFrame_, laserFrame_;

private:
    inline double nrand(double n) {
        return (n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX));
    }

    inline void xy2uv(double x, double y, int *u, int *v) {
        double dx = x - mapOrigin_.getX();
        double dy = y - mapOrigin_.getY();
        double yaw = -mapOrigin_.getYaw();
        double xx = dx * cos(yaw) - dy * sin(yaw);
        double yy = dx * sin(yaw) + dy * cos(yaw);
        *u = (int)(xx / mapResolution_);
        *v = (int)(yy / mapResolution_);
    }

    inline void uv2xy(int u, int v, double *x, double *y) {
        double xx = (double)u * mapResolution_;
        double yy = (double)v * mapResolution_;
        double yaw = mapOrigin_.getYaw();
        double dx = xx * cos(yaw) - yy * sin(yaw);
        double dy = xx * sin(yaw) + yy * cos(yaw);
        *x = dx + mapOrigin_.getX();
        *y = dy + mapOrigin_.getY();
    }

    void setMapInfo(nav_msgs::OccupancyGrid map);

    visualization_msgs::Marker makeSDFKeypointsMarker(std::vector<Keypoint> keypoints, std::string frame);

    // it does not correspond to rotation of the map
    void writeMapAndKeypoints(nav_msgs::OccupancyGrid map, std::vector<Keypoint> keypoints);

    void mapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    nav_msgs::OccupancyGrid buildLocalMap(void);

    std::vector<int> findCorrespondingFeatures(std::vector<Keypoint> localSDFKeypoints, std::vector<SDFOrientationFeature> localFeatures);

    double computeMatchingRate(Pose pose);

    geometry_msgs::PoseArray generatePoses(Pose currentOdomPose, std::vector<Keypoint> localSDFKeypoints,
        std::vector<SDFOrientationFeature> localSDFOrientationFeatures, std::vector<int> correspondingIndices);

    void scanCB(const sensor_msgs::LaserScan::ConstPtr &msg);
}; // class GLPoseSampler

} // namespace als_ros

#endif // __GL_POSE_SAMPLER_H__
