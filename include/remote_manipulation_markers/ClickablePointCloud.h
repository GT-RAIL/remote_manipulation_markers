#ifndef CLICKABLE_POINT_CLOUD_H_
#define CLICKABLE_POINT_CLOUD_H_

//ROS
#include <ros/ros.h>
#include <remote_manipulation_markers/GenerateGrasps.h>
#include <boost/thread/mutex.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <pcl_ros/point_cloud.h>
#include <remote_manipulation_markers/point_cloud_manipulation.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

//PCL
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/mls.h>

class ClickablePointCloud
{

public:

    ClickablePointCloud();

  void updateMarker();

private:
  void processInteractiveCloudFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &pc);

  ros::NodeHandle n;
  ros::NodeHandle pnh;

  //topics
  ros::Subscriber cloudSubscriber;
  ros::Publisher generatePublisher;
  ros::Publisher debugPublisher;
  ros::Publisher clickedPointPublisher;

  boost::mutex cloudMutex;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imServer;
  visualization_msgs::InteractiveMarker interactiveCloud;

  sensor_msgs::PointCloud2 cloud;

  bool newCloudReceived;  //flag for whether a new point cloud was received since the previous main loop execution
  bool cloudInitialized;  //flag for first point cloud received from a new topic

  tf::TransformListener tfListener;
};

#endif  // CLICKABLE_POINT_CLOUD_H_