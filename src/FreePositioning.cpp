#include <remote_manipulation_markers/FreePositioning.h>

using namespace std;

FreePositioning::FreePositioning() :
        pnh("~"), specifiedGraspServer(pnh, "execute_grasp", boost::bind(&FreePositioning::executeGraspCallback, this, _1), false)
{
  //read in parameters
  string graspTopic;
  pnh.param<string>("base_link", baseLink, "base_link");
  pnh.param<string>("eef_link", eefLink, "eef_link");
  pnh.param<string>("grasp_topic", graspTopic, "grasp");

  //messages
  markerPosePublisher = pnh.advertise<geometry_msgs::PoseStamped>("gripper_marker_pose", 1);

  //services
  resetMarkerPoseServer = pnh.advertiseService("reset_marker_pose", &FreePositioning::resetMarkerPoseCallback, this);

  //actionlib
  graspClient = new actionlib::SimpleActionClient<rail_manipulation_msgs::PickupAction>(graspTopic);

  imServer.reset(
      new interactive_markers::InteractiveMarkerServer("free_positioning", "free_positioning", false));

  ros::Duration(0.1).sleep();

  //set up marker
  tf::StampedTransform currentEefTransform;
  tfListener.waitForTransform(eefLink, baseLink, ros::Time::now(), ros::Duration(1.0));
  tfListener.lookupTransform(baseLink, eefLink, ros::Time(0), currentEefTransform);
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = currentEefTransform.getOrigin().x();
  pose.pose.position.y = currentEefTransform.getOrigin().y();
  pose.pose.position.z = currentEefTransform.getOrigin().z();
  pose.pose.orientation.x = currentEefTransform.getRotation().x();
  pose.pose.orientation.y = currentEefTransform.getRotation().y();
  pose.pose.orientation.z = currentEefTransform.getRotation().z();
  pose.pose.orientation.w = currentEefTransform.getRotation().w();
  visualization_msgs::InteractiveMarker iMarker = Common::makeGripperMarker(pose);
  iMarker.header.frame_id = baseLink;

  //add 6-DOF controls
  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  iMarker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  iMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  iMarker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  iMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  iMarker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  iMarker.controls.push_back(control);

  imServer->insert(iMarker);
  imServer->applyChanges();

  specifiedGraspServer.start();
}

FreePositioning::~FreePositioning()
{
  delete graspClient;
}

void FreePositioning::executeGraspCallback(const remote_manipulation_markers::SpecifiedGraspGoalConstPtr &goal)
{
  remote_manipulation_markers::SpecifiedGraspFeedback feedback;
  remote_manipulation_markers::SpecifiedGraspResult result;

  feedback.message = "Moving arm to your set position...";
  specifiedGraspServer.publishFeedback(feedback);

  visualization_msgs::InteractiveMarker poseMarker;
  imServer->get("gripper", poseMarker);

  rail_manipulation_msgs::PickupGoal graspGoal;
  graspGoal.pose.header = poseMarker.header;
  graspGoal.pose.pose = poseMarker.pose;
  graspGoal.lift = false;
  graspGoal.verify = false;
  graspClient->sendGoal(graspGoal);
  graspClient->waitForResult(ros::Duration(30.0));
  rail_manipulation_msgs::PickupResultConstPtr graspResult = graspClient->getResult();
  result.success = graspResult->success;
  result.executionSuccess = graspResult->executionSuccess;
  if (!graspResult->executionSuccess)
  {
    ROS_INFO("Grasp failed!");
    feedback.message = "Failed to plan to your grasp. Try a different start or end pose, and watch out for collisions.";
  }
  else
  {
    ROS_INFO("Grasp succeeded.");
    feedback.message = "Success!";
  }
  specifiedGraspServer.publishFeedback(feedback);
  specifiedGraspServer.setSucceeded(result);
}

bool FreePositioning::resetMarkerPoseCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  tf::StampedTransform currentEefTransform;
  tfListener.waitForTransform(eefLink, baseLink, ros::Time::now(), ros::Duration(1.0));
  tfListener.lookupTransform(baseLink, eefLink, ros::Time(0), currentEefTransform);
  geometry_msgs::Pose pose;
  pose.position.x = currentEefTransform.getOrigin().x();
  pose.position.y = currentEefTransform.getOrigin().y();
  pose.position.z = currentEefTransform.getOrigin().z();
  pose.orientation.x = currentEefTransform.getRotation().x();
  pose.orientation.y = currentEefTransform.getRotation().y();
  pose.orientation.z = currentEefTransform.getRotation().z();
  pose.orientation.w = currentEefTransform.getRotation().w();
  imServer->setPose("gripper", pose);
  imServer->applyChanges();

  return true;
}

void FreePositioning::publishMarkerPose()
{
  visualization_msgs::InteractiveMarker poseMarker;
  imServer->get("gripper", poseMarker);

  geometry_msgs::PoseStamped pose;
  pose.header = poseMarker.header;
  pose.pose = poseMarker.pose;

  markerPosePublisher.publish(pose);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "free_positioning");

  FreePositioning fp;

  ros::Rate loopRate(30);
  while (ros::ok())
  {
    fp.publishMarkerPose();
    ros::spinOnce();
    loopRate.sleep();
  }
}
