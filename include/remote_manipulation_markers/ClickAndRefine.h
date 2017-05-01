#ifndef CLICK_AND_REFINE_H_
#define CLICK_AND_REFINE_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/thread/recursive_mutex.hpp>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <rail_manipulation_msgs/PickupAction.h>
#include <remote_manipulation_markers/Common.h>
#include <remote_manipulation_markers/CreateSphere.h>
#include <remote_manipulation_markers/CycleGrasps.h>
#include <remote_manipulation_markers/ModeSwitch.h>
#include <remote_manipulation_markers/RefinablePose.h>
#include <remote_manipulation_markers/SpecifiedGraspAction.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_broadcaster.h>

class ClickAndRefine
{

public:

  /**
   * \brief Constructor
   */
  ClickAndRefine();

  ~ClickAndRefine();

  void publishMarkerPose();

  void publishGraspTransform();

private:

  bool clearGripperMarkerCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool clearFullMarkerCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool createSphereMarkerCallback(remote_manipulation_markers::CreateSphere::Request &req, remote_manipulation_markers::CreateSphere::Response &res);

  void processMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void executeGraspCallback(const remote_manipulation_markers::SpecifiedGraspGoalConstPtr &goal);

  void graspsCallback(const geometry_msgs::PoseArray &grasps);

  void updateMarker();

  bool cycleGraspsCallback(remote_manipulation_markers::CycleGrasps::Request &req, remote_manipulation_markers::CycleGrasps::Response &res);

  bool switchModeCallback(remote_manipulation_markers::ModeSwitch::Request &req, remote_manipulation_markers::ModeSwitch::Response &res);

  void enableMode(int mode);

  void cycleGraspsForward();

  void cycleGraspsBackward();

  void removeRefineMarkers();

  void addRefineMarkers();

  void translateGraspPoint(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void setApproachAngle(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void setWristRefinement(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  ros::NodeHandle n, pnh;

  //topics
  ros::Subscriber graspsSubscriber;
  ros::Publisher markerPosePublisher;

  //services
  ros::ServiceServer clearGripperMarkerServer;
  ros::ServiceServer clearFullMarkerServer;
  ros::ServiceServer createSphereServer;
  ros::ServiceServer cycleGraspsServer;
  ros::ServiceServer modeSwitchServer;

  //actionlib
  actionlib::SimpleActionClient<rail_manipulation_msgs::PickupAction> *graspClient;
  actionlib::SimpleActionServer<remote_manipulation_markers::SpecifiedGraspAction> specifiedGraspServer;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imServer; //!< interactive marker server

  boost::recursive_mutex graspsMutex;
  boost::recursive_mutex tfMutex;

  visualization_msgs::InteractiveMarker graspSelectorMarker;
  visualization_msgs::Marker graspMarker;
  visualization_msgs::InteractiveMarkerControl graspMarkerControl;

  geometry_msgs::PoseArray graspList;
  std::vector<RefinablePose> poses;
  geometry_msgs::TransformStamped graspPointTransform;
  geometry_msgs::TransformStamped graspAngleTransform;

  tf2_ros::TransformBroadcaster tfBroadcaster;

  bool graspsReceived;
  int graspIndex;
  int mode;
};

#endif
