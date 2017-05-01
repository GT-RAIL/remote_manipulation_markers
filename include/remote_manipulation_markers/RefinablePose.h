#ifndef REFINABLE_POSE_H_
#define REFINABLE_POSE_H_

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/InteractiveMarker.h>

class RefinablePose
{

public:

  RefinablePose();

  RefinablePose(geometry_msgs::PoseStamped pose);

  RefinablePose(geometry_msgs::PoseStamped pose, double initialDepth);

  void reset();

  double getDepth();

  geometry_msgs::PoseStamped refinedPose;
  geometry_msgs::Pose wristAdjustment;

private:
  geometry_msgs::PoseStamped originalPose;
  double originalDepth;
};

#endif
