#include <remote_manipulation_markers/TestClickAndRefine.h>

using namespace std;

TestClickAndRefine::TestClickAndRefine() :
    pnh("~")
{
  //messages
  graspPublisher = n.advertise<geometry_msgs::PoseArray>("grasp_sampler/sampled_grasps", 1);
}

void TestClickAndRefine::publishTestGrasps()
{
  ROS_INFO("Publishing test grasps...");
  geometry_msgs::PoseArray grasps;
  grasps.header.frame_id = "global_frame";
  grasps.header.stamp = ros::Time::now();
  geometry_msgs::Pose p1, p2, p3;
  p1.position.x = 1.0;
  p1.position.y = 0.5;
  p1.position.z = 0.25;
  p1.orientation.w = 1.0;
  p2.position.x = 1.0;
  p2.position.y = 0.4;
  p2.position.z = 0.2;
  p2.orientation.y = 0.7071;
  p2.orientation.w = 0.7071;
  p3.position.x = 1.1;
  p3.position.y = 0.45;
  p3.position.z = 0.3;
  p3.orientation.w = 1.0;
  grasps.poses.push_back(p1);
  grasps.poses.push_back(p2);
  grasps.poses.push_back(p3);
  graspPublisher.publish(grasps);
  ROS_INFO("Done");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "click_and_refine_test_node");

  TestClickAndRefine tpac;

  ros::Duration(1.0).sleep();

  tpac.publishTestGrasps();

  ros::spin();

  return EXIT_SUCCESS;
}
