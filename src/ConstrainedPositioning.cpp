#include <remote_manipulation_markers/ConstrainedPositioning.h>

using namespace std;

ConstrainedPositioning::ConstrainedPositioning() :
        pnh("~"), specifiedGraspServer(pnh, "execute_grasp", boost::bind(&ConstrainedPositioning::executeGraspCallback, this, _1), false)
{
  //read in parameters
  string graspTopic;
  pnh.param<string>("grasp_topic", graspTopic, "grasp");

  //messages
  markerPosePublisher = pnh.advertise<geometry_msgs::PoseStamped>("gripper_marker_pose", 1);

  //services
  clearGripperMarkerServer = pnh.advertiseService("clear_gripper_marker", &ConstrainedPositioning::clearGripperMarkerCallback, this);
  clearFullMarkerServer = pnh.advertiseService("clear_full_marker", &ConstrainedPositioning::clearFullMarkerCallback, this);
  createSphereServer = pnh.advertiseService("create_sphere", &ConstrainedPositioning::createSphereMarkerCallback, this);

  //actionlib
  graspClient = new actionlib::SimpleActionClient<rail_manipulation_msgs::PickupAction>(graspTopic);

  imServer.reset(
      new interactive_markers::InteractiveMarkerServer("constrained_positioning", "constrained_positioning", false));

  ros::Duration(0.1).sleep();

  imServer->applyChanges();

  specifiedGraspServer.start();
}

ConstrainedPositioning::~ConstrainedPositioning()
{
  delete graspClient;
}

bool ConstrainedPositioning::createSphereMarkerCallback(remote_manipulation_markers::CreateSphere::Request &req, remote_manipulation_markers::CreateSphere::Response &res)
{
  boost::recursive_mutex::scoped_lock lock(graspMutex);

  imServer->clear();

  visualization_msgs::InteractiveMarker sphere;
  sphere.header.frame_id = req.point.header.frame_id;
  sphere.pose.position.x = req.point.point.x;
  sphere.pose.position.y = req.point.point.y;
  sphere.pose.position.z = req.point.point.z;
  sphere.pose.orientation.w = 1.0;

  sphere.scale = 1.0;

  sphere.name = "cp_sphere";
  sphere.description = "Specify a grasp";

  visualization_msgs::Marker sphereMarker;
  sphereMarker.ns = "cp";
  sphereMarker.id = 0;
  sphereMarker.type = visualization_msgs::Marker::SPHERE;

  sphereMarker.pose.orientation.w = 1.0;
  sphereMarker.scale.x = 0.3;
  sphereMarker.scale.y = 0.3;
  sphereMarker.scale.z = 0.3;
  sphereMarker.color.r = 0.8;
  sphereMarker.color.g = 0.3;
  sphereMarker.color.b = 0.1;
  sphereMarker.color.a = 0.5;

  visualization_msgs::InteractiveMarkerControl sphereMarkerControl;
  sphereMarkerControl.name = "cp_sphere_control";
  sphereMarkerControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  sphereMarkerControl.always_visible = true;
  sphereMarkerControl.description = "Click to set an approach angle";
  sphereMarkerControl.markers.push_back(sphereMarker);

  visualization_msgs::Marker centerMarker;
  centerMarker.ns = "cp";
  centerMarker.id = 1;
  centerMarker.type = visualization_msgs::Marker::SPHERE;
  centerMarker.pose.orientation.w = 1.0;
  centerMarker.scale.x = 0.02;
  centerMarker.scale.y = 0.02;
  centerMarker.scale.z = 0.02;
  centerMarker.color.r = 0.0;
  centerMarker.color.g = 0.0;
  centerMarker.color.b = 1.0;
  centerMarker.color.a = 1.0;

  visualization_msgs::InteractiveMarkerControl centerMarkerControl;
  centerMarkerControl.name = "cp_center_control";
  centerMarkerControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  centerMarkerControl.always_visible = true;
  centerMarkerControl.description = "";
  centerMarkerControl.markers.push_back(centerMarker);

  sphere.controls.push_back(sphereMarkerControl);
  sphere.controls.push_back(centerMarkerControl);

  imServer->insert(sphere);
  imServer->setCallback(sphere.name, boost::bind(&ConstrainedPositioning::processMarkerFeedback, this, _1));
  imServer->applyChanges();

  return true;
}

void ConstrainedPositioning::processMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  boost::recursive_mutex::scoped_lock lock(graspMutex);
  if (feedback->control_name == "cp_sphere_control" && feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
  {
    if (feedback->mouse_point_valid)
    {
      visualization_msgs::InteractiveMarker cpSphere;
      imServer->get("cp_sphere", cpSphere);
      visualization_msgs::Marker sphere = cpSphere.controls[0].markers[0];
      double roll = 0;
      double pitch = -asin((cpSphere.pose.position.z - feedback->mouse_point.z)/sqrt(pow(feedback->mouse_point.x - cpSphere.pose.position.x, 2) + pow(feedback->mouse_point.y - cpSphere.pose.position.y, 2) + pow(feedback->mouse_point.z - cpSphere.pose.position.z, 2)));
      double yaw = atan2(cpSphere.pose.position.y - feedback->mouse_point.y, cpSphere.pose.position.x - feedback->mouse_point.x);
      //ROS_INFO("Mouse point: %f, %f, %f", feedback->mouse_point.x, feedback->mouse_point.y, feedback->mouse_point.z);
      //ROS_INFO("Sphere center: %f, %f, %f", cpSphere.pose.position.x, cpSphere.pose.position.y, cpSphere.pose.position.z);
      //ROS_INFO("Roll, Pitch, Yaw: %f, %f, %f", roll, pitch, yaw);
      geometry_msgs::QuaternionStamped orientation;
      orientation.quaternion = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
      orientation.header.frame_id = cpSphere.header.frame_id;

      visualization_msgs::InteractiveMarker gripperMarker;
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = orientation.header.frame_id;
      pose.pose.position = feedback->mouse_point;
      pose.pose.orientation = orientation.quaternion;
      if (imServer->get("gripper", gripperMarker))
      {
        //update gripper marker pose
        imServer->setPose("gripper", pose.pose, pose.header);
      }
      else
      {
        //create new gripper marker
        gripperMarker = Common::makeGripperMarker(pose);

        //add controls
        visualization_msgs::InteractiveMarkerControl control;

        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        gripperMarker.controls.push_back(control);

        control.name = "move_z";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        gripperMarker.controls.push_back(control);

        imServer->insert(gripperMarker);
      }
      imServer->applyChanges();
    }
    else
    {
      ROS_INFO("Invalid mouse point");
    }
  }
}

void ConstrainedPositioning::executeGraspCallback(const remote_manipulation_markers::SpecifiedGraspGoalConstPtr &goal)
{
  boost::recursive_mutex::scoped_lock lock(graspMutex);

  remote_manipulation_markers::SpecifiedGraspFeedback feedback;
  remote_manipulation_markers::SpecifiedGraspResult result;

  feedback.message = "Moving arm to your set position...";
  specifiedGraspServer.publishFeedback(feedback);

  visualization_msgs::InteractiveMarker poseMarker;
  if (!imServer->get("gripper", poseMarker))
  {
    feedback.message = "Please specify a grasp first.  See the instructions pane for details.";
    specifiedGraspServer.publishFeedback(feedback);
    result.success = false;
    result.executionSuccess = false;
    specifiedGraspServer.setSucceeded(result);
    return;
  }

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

bool ConstrainedPositioning::clearGripperMarkerCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  boost::recursive_mutex::scoped_lock lock(graspMutex);
  imServer->erase("gripper");
  imServer->applyChanges();

  return true;
}

bool ConstrainedPositioning::clearFullMarkerCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  boost::recursive_mutex::scoped_lock lock(graspMutex);
  imServer->clear();
  imServer->applyChanges();

  return true;
}

void ConstrainedPositioning::publishMarkerPose()
{
  visualization_msgs::InteractiveMarker poseMarker;
  if (imServer->get("gripper", poseMarker))
  {
    geometry_msgs::PoseStamped pose;
    pose.header = poseMarker.header;
    pose.pose = poseMarker.pose;

    markerPosePublisher.publish(pose);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "constrained_positioning");

  ConstrainedPositioning cp;

  ros::Rate loopRate(30);
  while (ros::ok())
  {
    cp.publishMarkerPose();
    ros::spinOnce();
    loopRate.sleep();
  }
}
