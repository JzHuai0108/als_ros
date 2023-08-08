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

#ifndef __MCL_H__
#define __MCL_H__

#include <string>
#include <vector>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <als_ros/Pose.h>
#include <als_ros/Particle.h>
#include <als_ros/MAEClassifier.h>

namespace als_ros {

class MCL {
private:
    // node handler
    ros::NodeHandle nh_;

    // subscribers
    std::string scanName_, odomName_, mapName_, glSampledPosesName_;
    ros::Subscriber scanSub_, odomSub_, mapSub_, glSampledPosesPub_, initialPoseSub_;

    // publishers
    std::string poseName_, particlesName_, unknownScanName_, residualErrorsName_, reliabilityName_, reliabilityMarkerName_;
    ros::Publisher posePub_, particlesPub_, unknownScanPub_, residualErrorsPub_, reliabilityPub_, reliabilityMarkerPub_;

    // tf frames
    std::string laserFrame_, baseLinkFrame_, mapFrame_, odomFrame_;
    bool broadcastTF_, useOdomTF_;

    // poses
    double initialPoseX_, initialPoseY_, initialPoseYaw_;
    Pose mclPose_, baseLink2Laser_, odomPose_;
    ros::Time mclPoseStamp_, odomPoseStamp_, glSampledPosesStamp_;

    // particles
    int particlesNum_;
    std::vector<Particle> particles_;
    double initialNoiseX_, initialNoiseY_, initialNoiseYaw_;
    bool useAugmentedMCL_, addRandomParticlesInResampling_;
    double randomParticlesRate_;
    std::vector<double> randomParticlesNoise_;
    int glParticlesNum_;
    std::vector<Particle> glParticles_;
    geometry_msgs::PoseArray glSampledPoses_;
    bool canUpdateGLSampledPoses_, canUseGLSampledPoses_, isGLSampledPosesUpdated_;
    double glSampledPoseTimeTH_, gmmPositionalVariance_, gmmAngularVariance_;
    double predDistUnifRate_;

    // map
    // cv::Mat distMap_;
    double mapResolution_;
    Pose mapOrigin_;
    int mapWidth_, mapHeight_;
    bool gotMap_;

    // motion
    double deltaX_, deltaY_, deltaDist_, deltaYaw_;
    double deltaXSum_, deltaYSum_, deltaDistSum_, deltaYawSum_, deltaTimeSum_;
    std::vector<double> resampleThresholds_;
    std::vector<double> odomNoiseDDM_, odomNoiseODM_;
    bool useOmniDirectionalModel_;

    // measurements
    sensor_msgs::LaserScan scan_, unknownScan_;
    bool canUpdateScan_;
    std::vector<bool> likelihoodShiftedSteps_;

    // measurement model
    // 0: likelihood field model, 1: beam model, 2: class conditional measurement model
    int measurementModelType_;
    double zHit_, zShort_, zMax_, zRand_;
    double varHit_, lambdaShort_, lambdaUnknown_;
    double normConstHit_, denomHit_, pRand_;
    double measurementModelRandom_, measurementModelInvalidScan_;
    double pKnownPrior_, pUnknownPrior_, unknownScanProbThreshold_;
    double alphaSlow_, alphaFast_, omegaSlow_, omegaFast_;
    int scanStep_;
    bool rejectUnknownScan_, publishUnknownScan_, publishResidualErrors_;
    bool gotScan_, scanMightInvalid_;
    double resampleThresholdESS_;

    // localization result
    double totalLikelihood_, averageLikelihood_, maxLikelihood_;
    double amclRandomParticlesRate_, effectiveSampleSize_;
    int maxLikelihoodParticleIdx_;

    // other parameters
    tf::TransformBroadcaster tfBroadcaster_;
    tf::TransformListener tfListener_;
    bool isInitialized_;
    double localizationHz_;
    double transformTolerance_;

    // reliability estimation
    bool estimateReliability_;
    int classifierType_;
    double reliability_;
    std::vector<double> reliabilities_, glSampledPosesReliabilities_;
    std::vector<double> relTransDDM_, relTransODM_;

    // mean absolute error (MAE)-based failure detector
    MAEClassifier maeClassifier_;
    std::string maeClassifierDir_;
    std::vector<double> maes_, glSampledPosesMAEs_;

    // global-localization-based pose sampling
    bool useGLPoseSampler_, fuseGLPoseSamplerOnlyUnreliable_;

    // constant parameters
    const double rad2deg_;

public:
    // inline setting functions
    inline void setCanUpdateScan(bool canUpdateScan) {
        canUpdateScan_ = canUpdateScan;
        if (!canUpdateScan_) {
            int invalidScanNum = 0;
            for (int i = 0; i < (int)scan_.ranges.size(); ++i) {
                double r = scan_.ranges[i];
                if (r < scan_.range_min || scan_.range_max < r)
                    invalidScanNum++;
            }
            double invalidScanRate = (double)invalidScanNum / (int)scan_.ranges.size();
            if (invalidScanRate > 0.95) {
                scanMightInvalid_ = true;
                ROS_ERROR("MCL scan might invalid.");
            } else {
                scanMightInvalid_ = false;
            }
        }
    }

    // inline getting functions
    inline double getLocalizationHz(void) { return localizationHz_; }
    inline std::string getMapFrame(void) { return mapFrame_; }
    inline sensor_msgs::LaserScan getScan(void) { return scan_; }
    inline int getParticlesNum(void) { return particlesNum_; }
    inline Pose getParticlePose(int i) { return particles_[i].getPose(); }
    inline double getParticleW(int i) { return particles_[i].getW(); }
    inline Pose getBaseLink2Laser(void) { return baseLink2Laser_; }
    inline int getScanStep(void) { return scanStep_; }
    inline int getMaxLikelihoodParticleIdx(void) { return maxLikelihoodParticleIdx_; }
    inline double getNormConstHit(void) { return normConstHit_; }
    inline double getDenomHit(void) { return denomHit_; }
    inline double getZHit(void) { return zHit_; }
    inline double getMeasurementModelRandom(void) { return measurementModelRandom_; }

    // inline setting functions
    inline void setMCLPoseStamp(ros::Time stamp) { mclPoseStamp_ = stamp; }
    inline void setParticleW(int i, double w) { particles_[i].setW(w); }
    inline void setTotalLikelihood(double totalLikelihood) { totalLikelihood_ = totalLikelihood; }
    inline void setAverageLikelihood(double averageLikelihood) { averageLikelihood_ = averageLikelihood; }
    inline void setMaxLikelihood(double maxLikelihood) { maxLikelihood_ = maxLikelihood; }
    inline void setMaxLikelihoodParticleIdx(int maxLikelihoodParticleIdx) { maxLikelihoodParticleIdx_ = maxLikelihoodParticleIdx; }

    // inline other functions
    void clearLikelihoodShiftedSteps(void) { likelihoodShiftedSteps_.clear(); }
    void addLikelihoodShiftedSteps(bool flag) { likelihoodShiftedSteps_.push_back(flag); }

    MCL(void);

    void updateParticlesByMotionModel(void);

    void calculateLikelihoodsByMeasurementModel(void);

    void calculateLikelihoodsByDecisionModel(void);

    void calculateGLSampledPosesLikelihood(void);

    void calculateAMCLRandomParticlesRate(void);

    void calculateEffectiveSampleSize(void);

    void estimatePose(void);

    void resampleParticles(void);

    template <typename T>
    std::vector<T> getResidualErrors(Pose pose);

    void printResult(void);

    void publishROSMessages(void);

    void broadcastTF(void);

    void plotLikelihoodMap(void);

private:
    inline double nrand(double n) { return (n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX)); }

    inline bool onMap(int u, int v) {
        if (0 <= u && u < mapWidth_ && 0 <= v && v < mapHeight_)
            return true;
        else
            return false;
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

    void scanCB(const sensor_msgs::LaserScan::ConstPtr &msg);

    void odomCB(const nav_msgs::Odometry::ConstPtr &msg);

    void mapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    void initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

    void glSampledPosesCB(const geometry_msgs::PoseArray::ConstPtr &msg);

    void resetParticlesDistribution(void);

    void resetReliabilities(void);

    void rejectUnknownScan(void);

    double calculateLikelihoodFieldModel(Pose pose, double range, double rangeAngle);

    double calculateBeamModel(Pose pose, double range, double rangeAngle);

    double calculateClassConditionalMeasurementModel(Pose pose, double range, double rangeAngle);

    void estimateUnknownScanWithClassConditionalMeasurementModel(Pose pose);

}; // class MCL

} // namespace als_ros

#endif // __MCL_H__
