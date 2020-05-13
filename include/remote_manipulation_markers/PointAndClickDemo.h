
#ifndef REMOTE_MANIPULATION_MARKERS_POINT_AND_CLICK_DEMO_H
#define REMOTE_MANIPULATION_MARKERS_POINT_AND_CLICK_DEMO_H

// Boost
#include <boost/thread/mutex.hpp>

// ROS
#include <actionlib/client/simple_action_client.h>
#include <remote_manipulation_markers/GenerateGrasps.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <rail_grasp_calculation_msgs/RankGraspsAction.h>
#include <rail_grasp_calculation_msgs/SampleGraspsAction.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class PointAndClickDemo
{

public:
    /**
     * @brief Initialize ROS message, services, and actions required for grasp suggestion.
     */
    PointAndClickDemo();

private:

    void generateCallback(const remote_manipulation_markers::GenerateGrasps::ConstPtr &msg);

    ros::NodeHandle n_, pnh_;

    // topics
    ros::Publisher grasps_publisher_;
    ros::Publisher best_grasp_publisher_;
    ros::Subscriber generate_subscriber_;

    // actionlib
    actionlib::SimpleActionClient<rail_grasp_calculation_msgs::SampleGraspsAction> sample_grasps_client_;
    actionlib::SimpleActionClient<rail_grasp_calculation_msgs::RankGraspsAction> rank_grasps_poi_client_;

    tf::TransformListener tf_listener_;

    double roi_radius_;
};

#endif  // REMOTE_MANIPULATION_MARKERS_POINT_AND_CLICK_DEMO_H