#include <remote_manipulation_markers/ClickablePointCloud.h>

using namespace std;

ClickablePointCloud::ClickablePointCloud() :
    pnh("~")
{
  ROS_INFO("Initializing interactive marker server...");

  // read parameters
  string suffix;
  pnh.param<string>("suffix", suffix, "");
  string cloudTopic;
  pnh.param<string>("cloud_topic", cloudTopic, "/camera/depth_registered/points");

  generatePublisher = n.advertise<remote_manipulation_markers::GenerateGrasps>("generate_grasps", 1);
//  debugPublisher = n.advertise<sensor_msgs::PointCloud2>("debug_clicked_cloud", 1);
  cloudSubscriber = n.subscribe(cloudTopic, 1, &ClickablePointCloud::cloudCallback, this);
  clickedPointPublisher = pnh.advertise<geometry_msgs::PointStamped>("clicked_point", 1);

  imServer.reset( new interactive_markers::InteractiveMarkerServer("clickable_point_cloud" + suffix, "clickable_point_markers",
                                                                   false));

  ros::Duration(0.1).sleep();

  imServer->applyChanges();

  interactiveCloud.pose.orientation.w = 1.0;
  interactiveCloud.scale = 1.0;
  interactiveCloud.name = "interactive_cloud_marker";
  interactiveCloud.description = "Clickable Point Cloud";

  newCloudReceived = false;

  ROS_INFO("Initialized.");
}

void ClickablePointCloud::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &pc)
{
  boost::mutex::scoped_lock lock(cloudMutex);

  cloud = *pc;

  newCloudReceived = true;
}

void ClickablePointCloud::processInteractiveCloudFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  boost::mutex::scoped_lock lock(cloudMutex);
  switch (feedback->event_type)
  {
    //Send a stop command so that when the marker is released the arm stops moving
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      if (feedback->mouse_point.x == 0 && feedback->mouse_point.y == 0 && feedback->mouse_point.z == 0)
      {
        ROS_INFO("invalid click!");
      }
      else
      {
        ROS_INFO("Clicked point (%f, %f, %f), in frame %s...", feedback->mouse_point.x, feedback->mouse_point.y, feedback->mouse_point.z, feedback->header.frame_id.c_str());

        geometry_msgs::PointStamped clickedPoint;
        clickedPoint.header.frame_id = feedback->header.frame_id;
        clickedPoint.point = feedback->mouse_point;

        clickedPointPublisher.publish(clickedPoint);

        //Crop point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr croppedCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr upsampledCloud(new pcl::PointCloud<pcl::PointXYZ>);
        PointCloudManipulation::fromSensorMsgs(cloud, pclCloud);
        float radius = 0.25f;
        pcl::CropBox<pcl::PointXYZ> cropBox;
        Eigen::Vector4f minPoint, maxPoint;
        {
          minPoint[0] = clickedPoint.point.x - radius;
          minPoint[1] = clickedPoint.point.y - radius;
          minPoint[2] = clickedPoint.point.z - radius;
          maxPoint[0] = clickedPoint.point.x + radius;
          maxPoint[1] = clickedPoint.point.y + radius;
          maxPoint[2] = clickedPoint.point.z + radius;
        }
        cropBox.setMin(minPoint);
        cropBox.setMax(maxPoint);
        cropBox.setInputCloud(pclCloud);
        cropBox.filter(*croppedCloud);

//        // point cloud approximate upsampling test code (warning: requires a lot of tuning!)
//        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
//
//        mls.setInputCloud(croppedCloud);
//        mls.setPolynomialFit(true);
//        mls.setPolynomialOrder(4);
//        mls.setSearchRadius(0.1);
//        mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::RANDOM_UNIFORM_DENSITY);
//        mls.setPointDensity(10000);
////        mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
////        mls.setUpsamplingRadius(0.01);
////        mls.setUpsamplingStepSize(0.0025);
////        mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::VOXEL_GRID_DILATION);
////        mls.setDilationVoxelSize(.005);
////        mls.setDilationIterations(3);
//        mls.setSearchMethod(tree);
//
//        mls.process(*upsampledCloud);

        remote_manipulation_markers::GenerateGrasps generate;
        generate.poi = clickedPoint;
//        PointCloudManipulation::toSensorMsgs(upsampledCloud, generate.cloud);
        PointCloudManipulation::toSensorMsgs(croppedCloud, generate.cloud);

//        debugPublisher.publish(generate.cloud);
        generatePublisher.publish(generate);
      }
      break;
    default:
      break;
  }

  //Update interactive marker server
  imServer->applyChanges();
}

void ClickablePointCloud::updateMarker()
{
  if (newCloudReceived)
  {
    boost::mutex::scoped_lock lock(cloudMutex);

    interactiveCloud.controls.clear();


    //create new marker
    visualization_msgs::Marker marker;
    // set header field
    marker.header.frame_id = cloud.header.frame_id;
    // default position
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // default scale
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    // set the type of marker and our color of choice
    marker.type = visualization_msgs::Marker::POINTS;


    // convert to an easy to use point cloud message
    sensor_msgs::PointCloud pc;
    sensor_msgs::convertPointCloud2ToPointCloud(cloud, pc);

    // place in the marker message
    marker.points.resize(pc.points.size());
    marker.colors.resize(pc.points.size());

    for (size_t j = 0; j < pc.points.size(); j++)
    {
      if (pc.points[j].x > -5.0 && pc.points[j].x < 5.0 && pc.points[j].y > -5.0 && pc.points[j].y < 5.0
          && pc.points[j].z > -5.0 && pc.points[j].z < 5.0)
      {
        marker.points[j].x = pc.points[j].x;
        marker.points[j].y = pc.points[j].y;
        marker.points[j].z = pc.points[j].z;
      }
      else
      {
        marker.points[j].x = 0;
        marker.points[j].y = 0;
        marker.points[j].z = 0;
      }
        // use average RGB
        uint32_t rgb = *reinterpret_cast<int *>(&pc.channels[0].values[j]);
        marker.colors[j].r = (float)((int) ((rgb >> 16) & 0x0000ff))/255.0f;
        marker.colors[j].g = (float)((int) ((rgb >> 8) & 0x0000ff))/255.0f;
        marker.colors[j].b = (float)((int) ((rgb) & 0x0000ff))/255.0f;
        marker.colors[j].a = 1.0;
    }

    //update interactive marker

    interactiveCloud.header.frame_id = cloud.header.frame_id;
    visualization_msgs::InteractiveMarkerControl clickControl;
    clickControl.markers.push_back(marker);
    clickControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    clickControl.name = "interactive_cloud_marker_control";

    interactiveCloud.header.frame_id = cloud.header.frame_id;
    interactiveCloud.controls.push_back(clickControl);
    imServer->clear();
    imServer->insert(interactiveCloud);
    imServer->setCallback(interactiveCloud.name, boost::bind(&ClickablePointCloud::processInteractiveCloudFeedback, this, _1));
    imServer->applyChanges();
    newCloudReceived = false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "clickable_point_cloud");

  ClickablePointCloud cpc;

  ros::Rate loop_rate(5);
  while (ros::ok())
  {
    cpc.updateMarker();
    ros::spinOnce();
    loop_rate.sleep();
  }
}