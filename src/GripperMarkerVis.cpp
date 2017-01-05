#include <remote_manipulation_markers/GripperMarkerVis.h>

using namespace std;

GripperMarkerVis::GripperMarkerVis() : pnh("~")
{
  string markerNodeName;
  //read in parameters
  pnh.param<string>("marker_node_name", markerNodeName, "free_positioning");

  //messages
  markerPoseSubscriber = n.subscribe(markerNodeName + "/gripper_marker_pose", 1, &GripperMarkerVis::markerPoseCallback, this);

  imServer.reset(
      new interactive_markers::InteractiveMarkerServer("nimbus_6dof_vis", "nimbus_6dof_vis", false));
  ros::Duration(0.1).sleep();

  imServer->applyChanges();
}

void GripperMarkerVis::markerPoseCallback(const geometry_msgs::PoseStamped& pose)
{
  visualization_msgs::InteractiveMarker gripperMarker;
  if (imServer->get("gripper", gripperMarker))
  {
    //update pose
    imServer->setPose("gripper", pose.pose);
  }
  else
  {
    //create new gripper vis marker with given pose
    gripperMarker = Common::makeGripperMarker(pose);
    imServer->insert(gripperMarker);
  }
  imServer->applyChanges();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gripper_marker_vis");

  GripperMarkerVis gmv;

  ros::spin();
}
