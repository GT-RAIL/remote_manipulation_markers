#ifndef FREE_POSITIONING_H_
#define FREE_POSITIONING_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <interactive_markers/interactive_marker_server.h>
#include <rail_manipulation_msgs/PickupAction.h>
#include <remote_manipulation_markers/Common.h>
#include <remote_manipulation_markers/SpecifiedGraspAction.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

class FreePositioning
{

public:

  /**
   * \brief Constructor
   */
  FreePositioning();

  ~FreePositioning();

  void publishMarkerPose();

private:

  void updateJoints(const sensor_msgs::JointState::ConstPtr& msg);

  bool resetMarkerPoseCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  void executeGraspCallback(const remote_manipulation_markers::SpecifiedGraspGoalConstPtr &goal);

  std::string baseLink;
  std::string eefLink;

  ros::NodeHandle n, pnh;

  //messages
  ros::Publisher markerPosePublisher;

  //services
  ros::ServiceServer resetMarkerPoseServer;

  //actionlib
  actionlib::SimpleActionClient<rail_manipulation_msgs::PickupAction> *graspClient;
  actionlib::SimpleActionServer<remote_manipulation_markers::SpecifiedGraspAction> specifiedGraspServer;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imServer; //!< interactive marker server

  tf::TransformListener tfListener;
};

#endif
