// map_localization_node
//
// Localizes the robot inside a prior ENU-aligned PCD map:
//   - FAST-LIO provides smooth odometry in the odom frame ("map" in the CMU stack)
//   - this node aligns the recent registered scans against the prior map with GICP
//     and publishes the drift correction as the prior_map -> odom TF (~1 Hz)
//   - RTK (bynav INSPVAX preferred, NavSatFix + motion alignment as fallback)
//     provides the global initial pose, and a translation-only fallback when
//     scan matching degenerates (open areas)
//
// The CMU local stack (terrain_analysis / local_planner) never sees the
// correction jumps: it keeps working in the odom frame.

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <novatel_oem7_msgs/INSPVAX.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <deque>
#include <mutex>
#include <cmath>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;

// ---------------- WGS84 -> local ENU ----------------

static void llaToEcef(double latDeg, double lonDeg, double alt,
                      double &x, double &y, double &z)
{
  const double a = 6378137.0;
  const double e2 = 6.69437999014e-3;
  double lat = latDeg * M_PI / 180.0, lon = lonDeg * M_PI / 180.0;
  double sLat = sin(lat), cLat = cos(lat), sLon = sin(lon), cLon = cos(lon);
  double N = a / sqrt(1.0 - e2 * sLat * sLat);
  x = (N + alt) * cLat * cLon;
  y = (N + alt) * cLat * sLon;
  z = (N * (1.0 - e2) + alt) * sLat;
}

struct EnuConverter {
  bool valid = false;
  Eigen::Vector3d ecef0;
  Eigen::Matrix3d R; // ECEF -> ENU

  void setOrigin(double latDeg, double lonDeg, double alt)
  {
    llaToEcef(latDeg, lonDeg, alt, ecef0.x(), ecef0.y(), ecef0.z());
    double lat = latDeg * M_PI / 180.0, lon = lonDeg * M_PI / 180.0;
    double sLat = sin(lat), cLat = cos(lat), sLon = sin(lon), cLon = cos(lon);
    R << -sLon,        cLon,        0,
         -sLat * cLon, -sLat * sLon, cLat,
          cLat * cLon,  cLat * sLon, sLat;
    valid = true;
  }

  Eigen::Vector3d toEnu(double latDeg, double lonDeg, double alt) const
  {
    Eigen::Vector3d p;
    llaToEcef(latDeg, lonDeg, alt, p.x(), p.y(), p.z());
    return R * (p - ecef0);
  }
};

// ---------------- node state ----------------

enum LocState { STATE_INIT = 0, STATE_TRACK_MAP = 1, STATE_TRACK_RTK = 2, STATE_LOST = 3 };

std::mutex dataMutex;

// parameters
std::string mapPath;
std::string priorMapFrame, odomFrame;
double scanVoxelSize, mapVoxelSize;
double accumDuration, matchPeriod;
double initFitnessMax, trackFitnessMax;
double initCorrDist, trackCorrDist;
double rtkStdMax, rtkTimeout, minTravel;
int minMatchPoints;
std::vector<double> antennaOffset;

// data
Eigen::Isometry3d odomPose = Eigen::Isometry3d::Identity(); // T_odom_body
double odomTime = -1;
bool odomReceived = false;

std::deque<std::pair<double, CloudT::Ptr>> scanQueue; // odom-frame registered scans

struct RtkSample {
  double time = -1;
  Eigen::Vector3d enu = Eigen::Vector3d::Zero(); // antenna position in prior_map
  double yaw = 0;          // ENU yaw of the body, valid only if hasYaw
  bool hasYaw = false;
  double horizStd = 1e9;
};
RtkSample lastRtk;

// motion-alignment bookkeeping (NavSatFix-only fallback)
bool travelRefSet = false;
Eigen::Vector3d travelRefEnu, travelRefOdom;

// estimate
LocState state = STATE_INIT;
Eigen::Isometry3d T_pm_odom = Eigen::Isometry3d::Identity();
bool tfValid = false;

EnuConverter enuConv;
CloudT::Ptr mapCloud(new CloudT());
pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;

ros::Publisher pubPose, pubFitness, pubStatus;
tf2_ros::TransformBroadcaster *tfBroadcaster = nullptr;

// ---------------- callbacks ----------------

void odomHandler(const nav_msgs::Odometry::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(dataMutex);
  const auto &p = msg->pose.pose;
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  odomPose = Eigen::Isometry3d::Identity();
  odomPose.linear() = q.toRotationMatrix();
  odomPose.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  odomTime = msg->header.stamp.toSec();
  odomReceived = true;
}

void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  CloudT::Ptr cloud(new CloudT());
  pcl::fromROSMsg(*msg, *cloud);

  CloudT::Ptr filtered(new CloudT());
  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);
  voxel.setInputCloud(cloud);
  voxel.filter(*filtered);

  double t = msg->header.stamp.toSec();
  std::lock_guard<std::mutex> lock(dataMutex);
  scanQueue.push_back({t, filtered});
  while (!scanQueue.empty() && t - scanQueue.front().first > accumDuration)
    scanQueue.pop_front();
}

void updateRtkSample(double t, double lat, double lon, double alt,
                     double horizStd, bool hasYaw, double yawEnu)
{
  if (!enuConv.valid) return;
  std::lock_guard<std::mutex> lock(dataMutex);
  lastRtk.time = t;
  lastRtk.enu = enuConv.toEnu(lat, lon, alt);
  lastRtk.horizStd = horizStd;
  lastRtk.hasYaw = hasYaw;
  lastRtk.yaw = yawEnu;

  // record a travel baseline for the yaw-from-motion fallback
  if (!hasYaw && horizStd < rtkStdMax && odomReceived) {
    if (!travelRefSet) {
      travelRefEnu = lastRtk.enu;
      travelRefOdom = odomPose.translation();
      travelRefSet = true;
    }
  }
}

void inspvaxHandler(const novatel_oem7_msgs::INSPVAX::ConstPtr &msg)
{
  double horizStd = std::hypot(msg->latitude_stdev, msg->longitude_stdev);
  // NovAtel/bynav azimuth: degrees, clockwise from true north -> ENU yaw
  double yawEnu = (90.0 - msg->azimuth) * M_PI / 180.0;
  updateRtkSample(msg->header.stamp.toSec(), msg->latitude, msg->longitude,
                  msg->height, horizStd, true, yawEnu);
}

void navsatHandler(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
  if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) return;
  double horizStd = std::sqrt(std::max(msg->position_covariance[0], 0.0) +
                              std::max(msg->position_covariance[4], 0.0));
  if (horizStd <= 0.0) horizStd = 1e9; // covariance not filled by the driver
  updateRtkSample(msg->header.stamp.toSec(), msg->latitude, msg->longitude,
                  msg->altitude, horizStd, false, 0.0);
}

// ---------------- helpers ----------------

CloudT::Ptr buildSourceCloud()
{
  CloudT::Ptr merged(new CloudT());
  for (const auto &item : scanQueue)
    *merged += *item.second;
  if (merged->empty()) return merged;

  CloudT::Ptr filtered(new CloudT());
  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);
  voxel.setInputCloud(merged);
  voxel.filter(*filtered);
  return filtered;
}

bool rtkFresh(double now) { return lastRtk.time > 0 && now - lastRtk.time < rtkTimeout; }
bool rtkGood(double now) { return rtkFresh(now) && lastRtk.horizStd < rtkStdMax; }

Eigen::Vector3d antennaToBody(const Eigen::Matrix3d &R_pm_body)
{
  return R_pm_body * Eigen::Vector3d(antennaOffset[0], antennaOffset[1], antennaOffset[2]);
}

// try to compose an initial guess of T_prior_map_odom from RTK
bool makeRtkGuess(double now, Eigen::Isometry3d &T_pm_odom_guess)
{
  if (!rtkGood(now) || !odomReceived) return false;

  double yaw;
  if (lastRtk.hasYaw) {
    yaw = lastRtk.yaw;
  } else {
    // yaw from motion: needs a travel baseline
    if (!travelRefSet) return false;
    Eigen::Vector3d dEnu = lastRtk.enu - travelRefEnu;
    Eigen::Vector3d dOdom = odomPose.translation() - travelRefOdom;
    if (dEnu.head<2>().norm() < minTravel || dOdom.head<2>().norm() < 0.5 * minTravel)
      return false;
    yaw = atan2(dEnu.y(), dEnu.x()) - atan2(dOdom.y(), dOdom.x());
  }

  Eigen::Isometry3d T_pm_body = Eigen::Isometry3d::Identity();
  T_pm_body.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  if (!lastRtk.hasYaw)
    T_pm_body.linear() = T_pm_body.linear() * odomPose.linear(); // odom attitude rotated by yaw offset
  T_pm_body.translation() = lastRtk.enu - antennaToBody(T_pm_body.linear());

  T_pm_odom_guess = T_pm_body * odomPose.inverse();
  return true;
}

bool runRegistration(const CloudT::Ptr &source, const Eigen::Isometry3d &guess,
                     double corrDist, Eigen::Isometry3d &result, double &fitness)
{
  gicp.setMaxCorrespondenceDistance(corrDist);
  gicp.setInputSource(source);
  CloudT aligned;
  gicp.align(aligned, guess.matrix().cast<float>());
  if (!gicp.hasConverged()) return false;
  fitness = gicp.getFitnessScore(corrDist); // mean squared distance of correspondences
  Eigen::Matrix4d m = gicp.getFinalTransformation().cast<double>();
  result = Eigen::Isometry3d(m);
  return true;
}

// ---------------- main loop ----------------

void matchTimerCallback(const ros::TimerEvent &)
{
  double now = ros::Time::now().toSec();

  CloudT::Ptr source;
  Eigen::Isometry3d odomPoseSnap;
  bool odomOk;
  {
    std::lock_guard<std::mutex> lock(dataMutex);
    source = buildSourceCloud();
    odomPoseSnap = odomPose;
    odomOk = odomReceived;
  }

  std_msgs::Int8 statusMsg;
  std_msgs::Float32 fitnessMsg;
  fitnessMsg.data = -1.0;

  if (state == STATE_INIT) {
    Eigen::Isometry3d guess;
    bool haveGuess;
    {
      std::lock_guard<std::mutex> lock(dataMutex);
      haveGuess = makeRtkGuess(now, guess);
    }
    if (haveGuess && (int)source->size() >= minMatchPoints) {
      Eigen::Isometry3d result;
      double fitness = -1.0;
      if (runRegistration(source, guess, initCorrDist, result, fitness) &&
          fitness < initFitnessMax) {
        std::lock_guard<std::mutex> lock(dataMutex);
        T_pm_odom = result;
        tfValid = true;
        state = STATE_TRACK_MAP;
        fitnessMsg.data = fitness;
        ROS_INFO("map_localization: global initialization done, fitness %.3f", fitness);
      } else {
        ROS_WARN_THROTTLE(5.0, "map_localization: init registration rejected (fitness %.3f)",
                          fitness);
      }
    } else {
      ROS_INFO_THROTTLE(5.0, "map_localization: waiting for RTK fix%s and scans (%zu pts)",
                        lastRtk.hasYaw ? "" : " / travel baseline", source->size());
    }
  } else {
    // tracking: refine T_pm_odom with the map, fall back to RTK translation
    bool updated = false;
    if ((int)source->size() >= minMatchPoints) {
      Eigen::Isometry3d result;
      double fitness = -1.0;
      if (runRegistration(source, T_pm_odom, trackCorrDist, result, fitness) &&
          fitness < trackFitnessMax) {
        std::lock_guard<std::mutex> lock(dataMutex);
        T_pm_odom = result;
        state = STATE_TRACK_MAP;
        updated = true;
        fitnessMsg.data = fitness;
      } else {
        fitnessMsg.data = fitness;
      }
    }
    if (!updated) {
      std::lock_guard<std::mutex> lock(dataMutex);
      if (rtkGood(now) && odomOk) {
        // translation-only correction, keep the last good rotation
        Eigen::Matrix3d R_pm_body = T_pm_odom.linear() * odomPoseSnap.linear();
        Eigen::Vector3d p_body = lastRtk.enu - R_pm_body *
            Eigen::Vector3d(antennaOffset[0], antennaOffset[1], antennaOffset[2]);
        T_pm_odom.translation() = p_body - T_pm_odom.linear() * odomPoseSnap.translation();
        state = STATE_TRACK_RTK;
      } else {
        state = STATE_LOST; // keep last transform
        ROS_WARN_THROTTLE(5.0, "map_localization: matching degenerated and no RTK, holding last correction");
      }
    }
  }

  statusMsg.data = (int8_t)state;
  pubStatus.publish(statusMsg);
  pubFitness.publish(fitnessMsg);

  if (tfValid) {
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.stamp = ros::Time::now();
    poseMsg.header.frame_id = priorMapFrame;
    Eigen::Isometry3d T_pm_body = T_pm_odom * odomPoseSnap;
    Eigen::Quaterniond q(T_pm_body.linear());
    poseMsg.pose.position.x = T_pm_body.translation().x();
    poseMsg.pose.position.y = T_pm_body.translation().y();
    poseMsg.pose.position.z = T_pm_body.translation().z();
    poseMsg.pose.orientation.w = q.w();
    poseMsg.pose.orientation.x = q.x();
    poseMsg.pose.orientation.y = q.y();
    poseMsg.pose.orientation.z = q.z();
    pubPose.publish(poseMsg);
  }
}

void tfTimerCallback(const ros::TimerEvent &)
{
  if (!tfValid) return;
  Eigen::Isometry3d T;
  {
    std::lock_guard<std::mutex> lock(dataMutex);
    T = T_pm_odom;
  }
  geometry_msgs::TransformStamped tfMsg;
  tfMsg.header.stamp = ros::Time::now();
  tfMsg.header.frame_id = priorMapFrame;
  tfMsg.child_frame_id = odomFrame;
  tfMsg.transform.translation.x = T.translation().x();
  tfMsg.transform.translation.y = T.translation().y();
  tfMsg.transform.translation.z = T.translation().z();
  Eigen::Quaterniond q(T.linear());
  tfMsg.transform.rotation.w = q.w();
  tfMsg.transform.rotation.x = q.x();
  tfMsg.transform.rotation.y = q.y();
  tfMsg.transform.rotation.z = q.z();
  tfBroadcaster->sendTransform(tfMsg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapLocalization");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate("~");

  nhPrivate.param<std::string>("map_path", mapPath, "");
  nhPrivate.param<std::string>("prior_map_frame", priorMapFrame, "prior_map");
  nhPrivate.param<std::string>("odom_frame", odomFrame, "map");
  nhPrivate.param<double>("scan_voxel_size", scanVoxelSize, 0.2);
  nhPrivate.param<double>("map_voxel_size", mapVoxelSize, 0.4);
  nhPrivate.param<double>("accum_duration", accumDuration, 3.0);
  nhPrivate.param<double>("match_period", matchPeriod, 1.0);
  nhPrivate.param<double>("init_fitness_max", initFitnessMax, 1.0);
  nhPrivate.param<double>("track_fitness_max", trackFitnessMax, 0.3);
  nhPrivate.param<double>("init_corr_dist", initCorrDist, 5.0);
  nhPrivate.param<double>("track_corr_dist", trackCorrDist, 2.0);
  nhPrivate.param<double>("rtk_std_max", rtkStdMax, 0.3);
  nhPrivate.param<double>("rtk_timeout", rtkTimeout, 2.0);
  nhPrivate.param<double>("min_travel", minTravel, 3.0);
  nhPrivate.param<int>("min_match_points", minMatchPoints, 100);
  nhPrivate.param<std::vector<double>>("antenna_offset", antennaOffset, {0.0, 0.0, 0.15});
  if (antennaOffset.size() != 3) antennaOffset = {0.0, 0.0, 0.15};

  double originLat, originLon, originAlt;
  bool haveOrigin = nhPrivate.getParam("origin_latitude", originLat) &&
                    nhPrivate.getParam("origin_longitude", originLon) &&
                    nhPrivate.getParam("origin_altitude", originAlt);
  if (haveOrigin) {
    enuConv.setOrigin(originLat, originLon, originAlt);
    ROS_INFO("map_localization: ENU origin %.8f, %.8f, %.2f", originLat, originLon, originAlt);
  } else {
    ROS_ERROR("map_localization: no ENU origin loaded (origin_latitude/longitude/altitude), "
              "RTK initialization disabled!");
  }

  if (mapPath.empty() || pcl::io::loadPCDFile<PointT>(mapPath, *mapCloud) < 0) {
    ROS_FATAL("map_localization: failed to load prior map '%s'", mapPath.c_str());
    return 1;
  }
  {
    CloudT::Ptr filtered(new CloudT());
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(mapVoxelSize, mapVoxelSize, mapVoxelSize);
    voxel.setInputCloud(mapCloud);
    voxel.filter(*filtered);
    mapCloud = filtered;
  }
  ROS_INFO("map_localization: prior map loaded, %zu points after voxel filter", mapCloud->size());

  gicp.setInputTarget(mapCloud);
  gicp.setMaximumIterations(50);
  gicp.setTransformationEpsilon(1e-6);

  ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry>("/Odometry", 50, odomHandler);
  ros::Subscriber subCloud = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered", 10, cloudHandler);
  ros::Subscriber subInspvax = nh.subscribe<novatel_oem7_msgs::INSPVAX>("/bynav/inspvax", 10, inspvaxHandler);
  ros::Subscriber subNavSat = nh.subscribe<sensor_msgs::NavSatFix>("/gps/fix", 10, navsatHandler);

  pubPose = nh.advertise<geometry_msgs::PoseStamped>("/localization/pose", 5);
  pubFitness = nh.advertise<std_msgs::Float32>("/localization/fitness", 5);
  pubStatus = nh.advertise<std_msgs::Int8>("/localization/status", 5);

  tf2_ros::TransformBroadcaster broadcaster;
  tfBroadcaster = &broadcaster;

  ros::Timer matchTimer = nh.createTimer(ros::Duration(matchPeriod), matchTimerCallback);
  ros::Timer tfTimer = nh.createTimer(ros::Duration(0.05), tfTimerCallback);

  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
