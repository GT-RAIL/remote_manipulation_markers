#ifndef CLICK_AND_REFINE_H_
#define CLICK_AND_REFINE_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseArray.h>

class TestClickAndRefine
{

public:

  TestClickAndRefine();

  void publishTestGrasps();

private:
  ros::NodeHandle n, pnh;

  //topics
  ros::Publisher graspPublisher;
};

#endif
