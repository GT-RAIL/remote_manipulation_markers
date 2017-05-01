#ifndef POINT_AND_CLICK_H_
#define POINT_AND_CLICK_H_

//ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/thread/recursive_mutex.hpp>
#include <geometry_msgs/PoseArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <rail_manipulation_msgs/PickupAction.h>
#include <remote_manipulation_markers/Common.h>
#include <remote_manipulation_markers/CycleGrasps.h>
#include <remote_manipulation_markers/SpecifiedGraspAction.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class PointAndClick
{

public:

  /**
   * \brief Constructor
   */
  PointAndClick();

  ~PointAndClick();

private:
  void graspsCallback(const geometry_msgs::PoseArray &grasps);

  void updateMarker();

  bool cycleGraspsCallback(remote_manipulation_markers::CycleGrasps::Request &req, remote_manipulation_markers::CycleGrasps::Response &res);

  void executeGraspCallback(const remote_manipulation_markers::SpecifiedGraspGoalConstPtr &goal);

  void cycleGraspsForward();

  void cycleGraspsBackward();

  ros::NodeHandle n;
  ros::NodeHandle pnh;

  //actionlib
  actionlib::SimpleActionClient<rail_manipulation_msgs::PickupAction> *graspClient;
  actionlib::SimpleActionServer<remote_manipulation_markers::SpecifiedGraspAction> specifiedGraspServer;

  //topics
  ros::Subscriber graspsSubscriber;

  //services
  ros::ServiceServer cycleGraspsServer;

  boost::recursive_mutex graspsMutex;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imServer;
  visualization_msgs::InteractiveMarker graspSelectorMarker;
  visualization_msgs::Marker graspMarker;
  visualization_msgs::InteractiveMarkerControl graspMarkerControl;

  geometry_msgs::PoseArray graspList;

  tf::TransformBroadcaster tfBroadcaster;
  tf::TransformListener tfListener;

  bool graspsReceived;
  int graspIndex;
};

#endif
