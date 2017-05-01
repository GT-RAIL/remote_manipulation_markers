#include <remote_manipulation_markers/RefinablePose.h>

using namespace std;

RefinablePose::RefinablePose()
{
  originalPose.pose.orientation.w = 1.0;
  originalDepth = 0;
  this->reset();
}

RefinablePose::RefinablePose(geometry_msgs::PoseStamped pose)
{
  originalPose = pose;
  originalDepth = 0;
  this->reset();
}

RefinablePose::RefinablePose(geometry_msgs::PoseStamped pose, double initialDepth)
{
  originalPose = pose;
  originalDepth = initialDepth;
  this->reset();
}

void RefinablePose::reset()
{
  refinedPose = originalPose;
  wristAdjustment.position.x = originalDepth;
  wristAdjustment.position.y = 0;
  wristAdjustment.position.z = 0;
  wristAdjustment.orientation.x = 0;
  wristAdjustment.orientation.y = 0;
  wristAdjustment.orientation.z = 0;
  wristAdjustment.orientation.w = 1;
}

double RefinablePose::getDepth()
{
  return wristAdjustment.position.x;
}
