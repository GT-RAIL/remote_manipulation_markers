#ifndef GRIPPER_MARKER_VIS_H_
#define GRIPPER_MARKER_VIS_H_

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <remote_manipulation_markers/Common.h>

class GripperMarkerVis
{

public:

  /**
   * \brief Constructor
   */
  GripperMarkerVis();

private:
  void markerPoseCallback(const geometry_msgs::PoseStamped& pose);

  ros::NodeHandle n, pnh;

  //messages
  ros::Subscriber markerPoseSubscriber;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imServer; //!< interactive marker server
};

#endif
