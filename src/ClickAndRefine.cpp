#include <remote_manipulation_markers/ClickAndRefine.h>

using namespace std;

ClickAndRefine::ClickAndRefine() :
    pnh("~"), specifiedGraspServer(pnh, "execute", boost::bind(&ClickAndRefine::executeCallback, this, _1), false)
{
  //read in parameters
  string graspTopic;
  string placeTopic;
  string calculatedPosesTopic;
  pnh.param<string>("grasp_topic", graspTopic, "grasp");
  pnh.param<string>("place_topic", placeTopic, "place");
  pnh.param<string>("calculated_poses_topic", calculatedPosesTopic, "grasp_sampler/sampled_grasps");
  pnh.param<string>("global_frame", globalFrame, "base_link");

  //messages
  graspsSubscriber = n.subscribe(calculatedPosesTopic, 1, &ClickAndRefine::graspsCallback, this);

  //services
  cycleGraspsServer = pnh.advertiseService("cycle_grasps", &ClickAndRefine::cycleGraspsCallback, this);
  modeSwitchServer = pnh.advertiseService("switch_mode", &ClickAndRefine::switchModeCallback, this);
  clearPosesServer = pnh.advertiseService("clear", &ClickAndRefine::clearPosesCallback, this);

  //actionlib
  graspClient = new actionlib::SimpleActionClient<rail_manipulation_msgs::PickupAction>(graspTopic);
  placeClient = new actionlib::SimpleActionClient<rail_manipulation_msgs::StoreAction>(placeTopic);

  imServer.reset( new interactive_markers::InteractiveMarkerServer("click_and_refine", "click_and_refine_server", false));
  ros::Duration(0.1).sleep();
  imServer->applyChanges();

  graspsReceived = false;
  graspIndex = 0;
  mode = remote_manipulation_markers::ModeSwitch::Request::VIEW;

  specifiedGraspServer.start();
}

ClickAndRefine::~ClickAndRefine()
{
  delete graspClient;
}

bool ClickAndRefine::clearPosesCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  graspList.poses.clear();
  poses.clear();
  graspsReceived = false;
  graspIndex = 0;

  imServer->clear();
  imServer->applyChanges();

  return true;
}

void ClickAndRefine::graspsCallback(const geometry_msgs::PoseArray &grasps)
{
  ROS_INFO("Receiving new grasps...");
  boost::recursive_mutex::scoped_lock lock(graspsMutex);

  if (grasps.poses.empty())
  {
    graspList.poses.clear();
    poses.clear();
    graspsReceived = false;
    graspIndex = 0;

    imServer->clear();
    imServer->applyChanges();
    return;
  }

  graspList = grasps;
  poses.clear();
  poses.resize(graspList.poses.size());
  for (unsigned int i = 0; i < graspList.poses.size(); i ++)
  {
    geometry_msgs::PoseStamped stampedPose, transformedPose;
    stampedPose.header.frame_id = graspList.header.frame_id;
    stampedPose.header.stamp = ros::Time(0);
    stampedPose.pose = graspList.poses[i];

    transformedPose.header.frame_id = globalFrame;
    transformedPose.header.stamp = ros::Time(0);
    tfListener.transformPose(globalFrame, stampedPose, transformedPose);

    RefinablePose newPose(transformedPose, -0.085);
    poses[i] = newPose;
  }
  graspIndex = 0;

  updateMarker();

  graspsReceived = true;

  ROS_INFO("Grasps received.");
}

void ClickAndRefine::updateMarker()
{
  boost::recursive_mutex::scoped_lock lock(tfMutex);

  ros::Time updateTime = ros::Time::now();

  graspPointTransform.header.frame_id = poses[graspIndex].refinedPose.header.frame_id;
  graspPointTransform.header.stamp = updateTime;
  graspPointTransform.child_frame_id = "displayed_grasp_point_frame";
  graspPointTransform.transform.translation.x = poses[graspIndex].refinedPose.pose.position.x;
  graspPointTransform.transform.translation.y = poses[graspIndex].refinedPose.pose.position.y;
  graspPointTransform.transform.translation.z = poses[graspIndex].refinedPose.pose.position.z;
  graspPointTransform.transform.rotation.w = 1.0;

  graspAngleTransform.header.frame_id = "displayed_grasp_point_frame";
  graspAngleTransform.header.stamp = updateTime;
  graspAngleTransform.child_frame_id = "displayed_grasp_angle_frame";
  graspAngleTransform.transform.rotation = poses[graspIndex].refinedPose.pose.orientation;

  tfBroadcaster.sendTransform(graspPointTransform);
  tfBroadcaster.sendTransform(graspAngleTransform);

  visualization_msgs::InteractiveMarker graspPointMarker;
  if (imServer->get("grasp_point", graspPointMarker))
  {
    geometry_msgs::Pose updatePose;
    updatePose.position = poses[graspIndex].refinedPose.pose.position;
    updatePose.orientation.w = 1.0;
    imServer->setPose("grasp_point", updatePose, poses[graspIndex].refinedPose.header);
  }
  else
  {
    ROS_INFO("Initializing grasp point marker");
    graspPointMarker.header.frame_id = poses[graspIndex].refinedPose.header.frame_id;
    graspPointMarker.pose.position = poses[graspIndex].refinedPose.pose.position;
    graspPointMarker.pose.orientation.w = 1.0;
    graspPointMarker.scale = 0.2;
    graspPointMarker.name = "grasp_point";
    graspPointMarker.description = "Grasp point";

    visualization_msgs::Marker centerMarker;
    centerMarker.ns = "grasp_point";
    centerMarker.id = 1;
    centerMarker.type = visualization_msgs::Marker::SPHERE;
    centerMarker.pose.orientation.w = 1.0;
    centerMarker.scale.x = 0.02;
    centerMarker.scale.y = 0.02;
    centerMarker.scale.z = 0.02;
    centerMarker.color.r = 0.0;
    centerMarker.color.g = 0.75;
    centerMarker.color.b = 1.0;
    centerMarker.color.a = 0.8;

    visualization_msgs::InteractiveMarkerControl centerMarkerControl;
    centerMarkerControl.name = "grasp_point_control";
    centerMarkerControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
    centerMarkerControl.always_visible = true;
    centerMarkerControl.description = "";
    centerMarkerControl.markers.push_back(centerMarker);

    graspPointMarker.controls.push_back(centerMarkerControl);

    imServer->insert(graspPointMarker);
  }

  visualization_msgs::InteractiveMarker graspAngleMarker;
  if (imServer->get("grasp_angle", graspAngleMarker))
  {
    imServer->erase("grasp_angle");
  }
  graspAngleMarker.header.frame_id = "displayed_grasp_point_frame";
  graspAngleMarker.header.stamp = ros::Time(0);
  graspAngleMarker.pose.orientation = poses[graspIndex].refinedPose.pose.orientation;
  graspAngleMarker.scale = 0.2;
  graspAngleMarker.name = "grasp_angle";
  graspAngleMarker.description = "Grasp Angle";

  visualization_msgs::Marker centerMarker;
  centerMarker.ns = "grasp_angle";
  centerMarker.id = 1;
  centerMarker.type = visualization_msgs::Marker::SPHERE;
  centerMarker.pose.orientation.w = 1.0;
  centerMarker.scale.x = 0.01;
  centerMarker.scale.y = 0.01;
  centerMarker.scale.z = 0.01;
  centerMarker.color.r = 0.2;
  centerMarker.color.g = 0.2;
  centerMarker.color.b = 0.2;
  centerMarker.color.a = 1.0;

  visualization_msgs::InteractiveMarkerControl centerMarkerControl;
  centerMarkerControl.name = "grasp_angle_control";
  centerMarkerControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  centerMarkerControl.always_visible = true;
  centerMarkerControl.description = "";
  centerMarkerControl.markers.push_back(centerMarker);

  graspAngleMarker.controls.push_back(centerMarkerControl);

  imServer->insert(graspAngleMarker);

  visualization_msgs::InteractiveMarker gripperMarker;
  geometry_msgs::PoseStamped gripperMarkerPose;
  gripperMarkerPose.header.frame_id = "displayed_grasp_angle_frame";
  gripperMarkerPose.header.stamp = ros::Time(0);
  gripperMarkerPose.pose = poses[graspIndex].wristAdjustment;
  gripperMarkerPose.pose.orientation.w = 1.0;
  if (imServer->get("gripper", gripperMarker))
  {
    //remove existing marker
    imServer->erase("gripper");
  }
  //create new gripper marker
  gripperMarker = Common::makeGripperMarker(gripperMarkerPose);
  imServer->insert(gripperMarker);

  imServer->applyChanges();
}

void ClickAndRefine::executeCallback(const remote_manipulation_markers::SpecifiedPoseGoalConstPtr &goal)
{
  remote_manipulation_markers::SpecifiedPoseResult result;
  remote_manipulation_markers::SpecifiedPoseFeedback feedback;

  if (!graspsReceived)
  {
    feedback.message = "No grasp to execute!";
    specifiedGraspServer.publishFeedback(feedback);

    result.executionSuccess = false;
    result.success = false;

    specifiedGraspServer.setSucceeded(result);
    return;
  }

  if (goal->action == remote_manipulation_markers::SpecifiedPoseGoal::GRASP)
  {
    feedback.message = "The robot is attempting to move to your grasp position...";
    specifiedGraspServer.publishFeedback(feedback);

    rail_manipulation_msgs::PickupGoal graspGoal;
    graspGoal.pose.header.frame_id = "displayed_grasp_angle_frame";
    graspGoal.pose.pose.position.x = poses[graspIndex].getDepth() + 0.085;
    graspGoal.pose.pose.orientation.w = 1.0;
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
      feedback.message = "The robot failed to plan to your selected grasp.";
    }
    else
    {
      ROS_INFO("Grasp succeeded.");
      feedback.message = "Grasp successful!";
    }
  }
  else if (goal->action == remote_manipulation_markers::SpecifiedPoseGoal::PLACE)
  {
    feedback.message = "The robot is attempting to move to your place position...";
    specifiedGraspServer.publishFeedback(feedback);

    rail_manipulation_msgs::StoreGoal placeGoal;
    placeGoal.store_pose.header.frame_id = "displayed_grasp_angle_frame";
    placeGoal.store_pose.pose.position.x = poses[graspIndex].getDepth();
    placeGoal.store_pose.pose.orientation.w = 1.0;
    placeClient->sendGoal(placeGoal);
    placeClient->waitForResult(ros::Duration(30.0));
    rail_manipulation_msgs::StoreResultConstPtr placeResult = placeClient->getResult();
    result.success = placeResult->success;
    if (!placeResult->success)
    {
      ROS_INFO("Place failed!");
      feedback.message = "The robot failed place action at your specified location.";
    }
    else
    {
      ROS_INFO("Place succeeded.");
      feedback.message = "Place successful!";
    }
  }

  specifiedGraspServer.publishFeedback(feedback);
  specifiedGraspServer.setSucceeded(result);
}

bool ClickAndRefine::cycleGraspsCallback(remote_manipulation_markers::CycleGrasps::Request &req, remote_manipulation_markers::CycleGrasps::Response &res)
{
  boost::recursive_mutex::scoped_lock lock(graspsMutex);

  if (!graspsReceived)
  {
    ROS_INFO("No grasps to cycle through!");

    return false;
  }

  if (mode == remote_manipulation_markers::ModeSwitch::Request::VIEW)
  {
    if (req.forward)
    {
      if (graspIndex < graspList.poses.size() - 1)
        cycleGraspsForward();
    }
    else
    {
      if (graspIndex > 0)
        cycleGraspsBackward();
    }
  }
  else
  {
    ROS_INFO("Switch to view mode to cycle through the grasp list!");
  }

  res.grasp = poses[graspIndex].refinedPose;
  res.index = graspIndex;

  return true;
}

bool ClickAndRefine::switchModeCallback(remote_manipulation_markers::ModeSwitch::Request &req, remote_manipulation_markers::ModeSwitch::Response &res)
{
  visualization_msgs::InteractiveMarker gripperMarker;
  if (req.mode == remote_manipulation_markers::ModeSwitch::Request::VIEW || imServer->get("gripper", gripperMarker))
  {
    enableMode(req.mode);
    res.success = true;
  }
  else
  {
    ROS_INFO("No active grasp to refine, switching to view mode");
    enableMode(remote_manipulation_markers::ModeSwitch::Request::VIEW);
    res.success = false;
  }
}

void ClickAndRefine::enableMode(int newMode)
{
  mode = newMode;

  removeRefineMarkers();
  if (mode != remote_manipulation_markers::ModeSwitch::Request::VIEW)
    addRefineMarkers();
  imServer->applyChanges();
}

void ClickAndRefine::removeRefineMarkers()
{
  ROS_INFO("Removing any refine markers...");

  visualization_msgs::InteractiveMarker gripperMarker;
  visualization_msgs::InteractiveMarker graspPointMarker;
  visualization_msgs::InteractiveMarker graspAngleMarker;
  if (imServer->get("gripper", gripperMarker))
  {
    bool update = false;
    for (vector<visualization_msgs::InteractiveMarkerControl>::iterator it = gripperMarker.controls.end() - 1; it >= gripperMarker.controls.begin(); it--)
    {
      if (it->name == "depth" || it->name == "roll")
      {
        gripperMarker.controls.erase(it);
        update = true;
      }
    }

    if (update)
    {
      imServer->erase("gripper");
      imServer->insert(gripperMarker);
    }
  }

  if (imServer->get("grasp_point", graspPointMarker))
  {
    bool update = false;
    for (vector<visualization_msgs::InteractiveMarkerControl>::iterator it = graspPointMarker.controls.end() - 1; it >= graspPointMarker.controls.begin(); it--)
    {
      if (it->name == "grasp_point_x" || it->name == "grasp_point_y" || it->name == "grasp_point_z")
      {
        graspPointMarker.controls.erase(it);
        update = true;
      }
    }

    if (update)
    {
      imServer->erase("grasp_point");
      imServer->insert(graspPointMarker);
    }
  }

  if (imServer->get("grasp_angle", graspAngleMarker))
  {
    bool update = false;
    for (vector<visualization_msgs::InteractiveMarkerControl>::iterator it = graspAngleMarker.controls.end() - 1; it >= graspAngleMarker.controls.begin(); it--)
    {
      if (it->name == "grasp_angle_pitch" || it->name == "grasp_angle_yaw")
      {
        graspAngleMarker.controls.erase(it);
        update = true;
      }
    }

    if (update)
    {
      imServer->erase("grasp_angle");
      imServer->insert(graspAngleMarker);
    }
  }
}

void ClickAndRefine::addRefineMarkers()
{
  visualization_msgs::InteractiveMarker gripperMarker;
  visualization_msgs::InteractiveMarker graspPointMarker;
  visualization_msgs::InteractiveMarker graspAngleMarker;

  switch (mode)
  {
    case remote_manipulation_markers::ModeSwitch::Request::REFINE_POINT:
      //enable 3-dof translation markers on grasp point marker
      if (imServer->get("grasp_point", graspPointMarker))
      {
        ROS_INFO("Adding grasp point transformation markers...");

        visualization_msgs::InteractiveMarkerControl graspPointX, graspPointY, graspPointZ;
        graspPointX.orientation.w = 1;
        graspPointX.orientation.x = 1;
        graspPointX.orientation.y = 0;
        graspPointX.orientation.z = 0;
        graspPointX.name = "grasp_point_x";
        graspPointX.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

        graspPointY.orientation.w = 1;
        graspPointY.orientation.x = 0;
        graspPointY.orientation.y = 1;
        graspPointY.orientation.z = 0;
        graspPointY.name = "grasp_point_y";
        graspPointY.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

        graspPointZ.orientation.w = 1;
        graspPointZ.orientation.x = 0;
        graspPointZ.orientation.y = 0;
        graspPointZ.orientation.z = 1;
        graspPointZ.name = "grasp_point_z";
        graspPointZ.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

        graspPointMarker.controls.push_back(graspPointX);
        graspPointMarker.controls.push_back(graspPointY);
        graspPointMarker.controls.push_back(graspPointZ);

        imServer->erase(graspPointMarker.name);
        imServer->insert(graspPointMarker);
        imServer->setCallback(graspPointMarker.name, boost::bind(&ClickAndRefine::translateGraspPoint, this, _1), visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
      }

    break;
    case remote_manipulation_markers::ModeSwitch::Request::REFINE_APPROACH_ANGLE:
      //enable 2-dof approach angle adjustment markers on gripper marker
      if (imServer->get("grasp_angle", graspAngleMarker))
      {
        ROS_INFO("Adding approach angle adjustment markers...");

        visualization_msgs::InteractiveMarkerControl approachAnglePitch, approachAngleYaw;
        approachAnglePitch.orientation.w = 1;
        approachAnglePitch.orientation.x = 0;
        approachAnglePitch.orientation.y = 1;
        approachAnglePitch.orientation.z = 0;
        approachAnglePitch.name = "grasp_angle_pitch";
        approachAnglePitch.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;

        approachAngleYaw.orientation.w = 1;
        approachAngleYaw.orientation.x = 0;
        approachAngleYaw.orientation.y = 0;
        approachAngleYaw.orientation.z = 1;
        approachAngleYaw.name = "grasp_angle_yaw";
        approachAngleYaw.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;

        visualization_msgs::Marker pitchMarkerLeft;
        //pitchMarkerLeft.header.frame_id = "displayed_grasp_angle_frame";
        pitchMarkerLeft.ns = "grasp_angle_pitch";
        pitchMarkerLeft.id = 1;
        pitchMarkerLeft.points.resize(2);
        pitchMarkerLeft.points[0].x = poses[graspIndex].getDepth();
        pitchMarkerLeft.points[0].y = .1;
        pitchMarkerLeft.points[0].z = 0;
        pitchMarkerLeft.points[1].x = poses[graspIndex].getDepth() + 0.05;
        pitchMarkerLeft.points[1].y = .16;
        pitchMarkerLeft.points[1].z = 0;
        pitchMarkerLeft.type = visualization_msgs::Marker::ARROW;
        pitchMarkerLeft.color.r = 1.0;
        pitchMarkerLeft.color.g = 0.5;
        pitchMarkerLeft.color.b = 0.0;
        pitchMarkerLeft.color.a = 1.0;
        pitchMarkerLeft.scale.x = 0.03;
        pitchMarkerLeft.scale.y = 0.05;
        pitchMarkerLeft.scale.z = 0.03;
        approachAnglePitch.markers.push_back(pitchMarkerLeft);
        visualization_msgs::Marker pitchMarkerRight = pitchMarkerLeft;
        pitchMarkerRight.id = 2;
        pitchMarkerRight.points[0].y *= -1;
        pitchMarkerRight.points[1].y *= -1;
        approachAnglePitch.markers.push_back(pitchMarkerRight);

        visualization_msgs::Marker yawMarkerLeft;
        //yawMarkerLeft.header.frame_id = "displayed_grasp_angle_frame";
        yawMarkerLeft.ns = "grasp_angle_yaw";
        yawMarkerLeft.id = 1;
        yawMarkerLeft.points.resize(2);
        yawMarkerLeft.points[0].x = poses[graspIndex].getDepth();
        yawMarkerLeft.points[0].y = 0;
        yawMarkerLeft.points[0].z = .1;
        yawMarkerLeft.points[1].x = poses[graspIndex].getDepth() + 0.05;
        yawMarkerLeft.points[1].y = 0;
        yawMarkerLeft.points[1].z = .16;
        yawMarkerLeft.type = visualization_msgs::Marker::ARROW;
        yawMarkerLeft.color.r = 0.75;
        yawMarkerLeft.color.g = 1.0;
        yawMarkerLeft.color.b = 0.0;
        yawMarkerLeft.color.a = 1.0;
        yawMarkerLeft.scale.x = 0.03;
        yawMarkerLeft.scale.y = 0.05;
        yawMarkerLeft.scale.z = 0.03;
        approachAngleYaw.markers.push_back(yawMarkerLeft);
        visualization_msgs::Marker yawMarkerRight = yawMarkerLeft;
        yawMarkerRight.id = 2;
        yawMarkerRight.points[0].z *= -1;
        yawMarkerRight.points[1].z *= -1;
        approachAngleYaw.markers.push_back(yawMarkerRight);

        graspAngleMarker.controls.push_back(approachAnglePitch);
        graspAngleMarker.controls.push_back(approachAngleYaw);

        imServer->erase(graspAngleMarker.name);
        imServer->insert(graspAngleMarker);
        imServer->setCallback(graspAngleMarker.name, boost::bind(&ClickAndRefine::setApproachAngle, this, _1), visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);

      }
    break;
    case remote_manipulation_markers::ModeSwitch::Request::REFINE_WRIST:
      //enable wrist rotation marker and depth marker on gripper marker
      if (imServer->get("gripper", gripperMarker))
      {
        ROS_INFO("Adding wrist rotation and grasp depth markers...");
        visualization_msgs::InteractiveMarkerControl depthAdjustment, wristRoll;
        depthAdjustment.orientation.w = 1;
        depthAdjustment.orientation.x = 1;
        depthAdjustment.orientation.y = 0;
        depthAdjustment.orientation.z = 0;
        depthAdjustment.name = "depth";
        depthAdjustment.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        wristRoll.orientation.w = 1;
        wristRoll.orientation.x = 1;
        wristRoll.orientation.y = 0;
        wristRoll.orientation.z = 0;
        wristRoll.name = "roll";
        wristRoll.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;

        gripperMarker.controls.push_back(depthAdjustment);
        gripperMarker.controls.push_back(wristRoll);

        imServer->erase(gripperMarker.name);
        imServer->insert(gripperMarker);
        imServer->setCallback(gripperMarker.name, boost::bind(&ClickAndRefine::setWristRefinement, this, _1), visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
      }
    break;
  }
}

void ClickAndRefine::setWristRefinement(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  boost::recursive_mutex::scoped_lock lock(tfMutex);
  boost::recursive_mutex::scoped_lock lock2(graspsMutex);

  ros::Time updateTime = ros::Time::now();

  poses[graspIndex].refinedPose.header.stamp = ros::Time(0);
  //ROS_INFO("Feedback pose header: %s", feedback->header.frame_id.c_str());
  poses[graspIndex].wristAdjustment = feedback->pose;
}

void ClickAndRefine::setApproachAngle(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  boost::recursive_mutex::scoped_lock lock(tfMutex);
  boost::recursive_mutex::scoped_lock lock2(graspsMutex);

  ros::Time updateTime = ros::Time::now();

  //Update stored refinedPose
  poses[graspIndex].refinedPose.header.stamp = ros::Time(0);
  poses[graspIndex].refinedPose.pose.orientation = feedback->pose.orientation;

  //Update grasp transform
  graspAngleTransform.header.stamp = updateTime;
  graspAngleTransform.transform.rotation = feedback->pose.orientation;

  tfBroadcaster.sendTransform(graspAngleTransform);
}

void ClickAndRefine::translateGraspPoint(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  boost::recursive_mutex::scoped_lock lock(tfMutex);
  boost::recursive_mutex::scoped_lock lock2(graspsMutex);

  ros::Time updateTime = ros::Time::now();

  //Update stored refinedPose
  poses[graspIndex].refinedPose.header.stamp = ros::Time(0);
  poses[graspIndex].refinedPose.pose.position = feedback->pose.position;

  //Update grasp transform
  graspPointTransform.header.stamp = updateTime;
  graspPointTransform.transform.translation.x = feedback->pose.position.x;
  graspPointTransform.transform.translation.y = feedback->pose.position.y;
  graspPointTransform.transform.translation.z = feedback->pose.position.z;

  tfBroadcaster.sendTransform(graspPointTransform);

  //Update interactive marker pose
  //imServer->setPose(feedback->marker_name, feedback->pose);
  //imServer->applyChanges();
}

void ClickAndRefine::cycleGraspsForward()
{
  graspIndex ++;
  updateMarker();
}

void ClickAndRefine::cycleGraspsBackward()
{
  graspIndex --;
  updateMarker();
}

void ClickAndRefine::publishGraspTransform()
{
  boost::recursive_mutex::scoped_lock lock(tfMutex);
  visualization_msgs::InteractiveMarker gripperMarker;
  if (imServer->get("gripper", gripperMarker))
  {
    ros::Time updateTime = ros::Time::now();
    graspPointTransform.header.stamp = updateTime;
    graspAngleTransform.header.stamp = updateTime;
    tfBroadcaster.sendTransform(graspPointTransform);
    tfBroadcaster.sendTransform(graspAngleTransform);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "click_and_refine");

  ClickAndRefine pac;

  ros::Rate loopRate(30);
  while(ros::ok())
  {
    pac.publishGraspTransform();
    ros::spinOnce();
    loopRate.sleep();
  }

  return EXIT_SUCCESS;
}
