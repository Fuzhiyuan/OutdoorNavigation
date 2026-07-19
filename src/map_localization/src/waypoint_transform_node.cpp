// waypoint_transform_node
//
// Bridges global goals into the CMU local stack:
//   subscribes /way_point_prior (geometry_msgs/PointStamped in the prior_map frame)
//   and republishes it as /way_point in the odom frame ("map"), using the
//   prior_map -> odom TF from map_localization.
//
// The goal is re-transformed and re-published at 1 Hz so that later drift
// corrections (prior_map -> odom updates) keep pulling the goal to the right
// place without the local planner ever seeing a pose jump.

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

std::string priorMapFrame, odomFrame;
tf2_ros::Buffer tfBuffer;
ros::Publisher pubWaypoint;

geometry_msgs::PointStamped goalPrior;
bool goalReceived = false;

void publishGoal()
{
  if (!goalReceived) return;
  geometry_msgs::TransformStamped tfMsg;
  try {
    tfMsg = tfBuffer.lookupTransform(odomFrame, priorMapFrame, ros::Time(0), ros::Duration(0.1));
  } catch (tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(5.0, "waypoint_transform: no %s -> %s TF yet: %s",
                      priorMapFrame.c_str(), odomFrame.c_str(), ex.what());
    return;
  }
  geometry_msgs::PointStamped goalOdom;
  tf2::doTransform(goalPrior, goalOdom, tfMsg);
  goalOdom.header.stamp = ros::Time::now();
  goalOdom.header.frame_id = odomFrame;
  pubWaypoint.publish(goalOdom);
}

void goalHandler(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  goalPrior = *msg;
  goalPrior.header.frame_id = priorMapFrame; // enforce, header may be empty
  goalReceived = true;
  publishGoal();
}

void timerCallback(const ros::TimerEvent &)
{
  publishGoal();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypointTransform");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate("~");

  nhPrivate.param<std::string>("prior_map_frame", priorMapFrame, "prior_map");
  nhPrivate.param<std::string>("odom_frame", odomFrame, "map");

  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PointStamped>("/way_point_prior", 5, goalHandler);
  pubWaypoint = nh.advertise<geometry_msgs::PointStamped>("/way_point", 5);

  ros::Timer timer = nh.createTimer(ros::Duration(1.0), timerCallback);

  ros::spin();
  return 0;
}
