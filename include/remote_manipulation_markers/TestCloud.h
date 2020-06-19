#ifndef TEST_CLOUD_H_
#define TEST_CLOUD_H_

//ROS
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <pcl_ros/point_cloud.h>
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

class TestCloud
{

public:

    TestCloud();

  void publishCloud();

private:
  ros::NodeHandle n;
  ros::NodeHandle pnh;

  //topics
  ros::Publisher cloudPublisher;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr;
  sensor_msgs::PointCloud2 cloud;
};

#endif  // TEST_CLOUD_H_