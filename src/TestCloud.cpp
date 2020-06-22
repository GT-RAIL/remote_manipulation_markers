#include <remote_manipulation_markers/TestCloud.h>

using namespace std;

TestCloud::TestCloud() :
    pnh("~"), cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>)
{

  cloudPublisher = n.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("test_cloud", 1);

  // create point cloud object
  pcl::PointCloud<pcl::PointXYZRGB> myCloud;

  // fill cloud with random points
  for (int v=0; v<5000; ++v)
  {
    pcl::PointXYZRGB newPoint;
    newPoint.z = (rand() * 0.4) / RAND_MAX;
    newPoint.y = (rand() * 0.6) / RAND_MAX - 0.3;
    newPoint.x = 0.5 + 0.01*(sin(newPoint.z*150));
    newPoint.r = 150.0;
    newPoint.g = 150.0;
    newPoint.b = 150.0;
    myCloud.points.push_back(newPoint);
  }

  *cloudPtr = myCloud;
  cloudPtr->header.frame_id = "cloud_frame";

  ROS_INFO("Point cloud initialized.");
}

void TestCloud::publishCloud()
{
  cloudPublisher.publish(cloudPtr);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_cloud");

  TestCloud tc;

  ros::Rate loop_rate(5);
  while (ros::ok())
  {
    tc.publishCloud();
    ros::spinOnce();
    loop_rate.sleep();
  }
}