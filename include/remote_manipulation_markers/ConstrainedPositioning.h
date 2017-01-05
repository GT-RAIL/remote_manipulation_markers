#ifndef CONSTRAINED_POSITIONING_H_
#define CONSTRAINED_POSITIONING_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/thread/recursive_mutex.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <remote_manipulation_markers/Common.h>
#include <remote_manipulation_markers/CreateSphere.h>
#include <remote_manipulation_markers/SpecifiedGraspAction.h>
#include <rail_manipulation_msgs/PickupAction.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>

class ConstrainedPositioning
{

public:

  /**
   * \brief Constructor
   */
  ConstrainedPositioning();

  ~ConstrainedPositioning();

  void publishMarkerPose();

private:

  bool clearGripperMarkerCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool clearFullMarkerCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool createSphereMarkerCallback(remote_manipulation_markers::CreateSphere::Request &req, remote_manipulation_markers::CreateSphere::Response &res);

  void processMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void executeGraspCallback(const remote_manipulation_markers::SpecifiedGraspGoalConstPtr &goal);

  ros::NodeHandle n, pnh;

  //publishers
  ros::Publisher markerPosePublisher;

  //services
  ros::ServiceServer clearGripperMarkerServer;
  ros::ServiceServer clearFullMarkerServer;
  ros::ServiceServer createSphereServer;

  //actionlib
  actionlib::SimpleActionClient<rail_manipulation_msgs::PickupAction> *graspClient;
  actionlib::SimpleActionServer<remote_manipulation_markers::SpecifiedGraspAction> specifiedGraspServer;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imServer; //!< interactive marker server

  boost::recursive_mutex graspMutex;
};

#endif
