#ifndef COMMON_H_
#define COMMON_H_

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/InteractiveMarker.h>

class Common
{

public:

  Common();

  static visualization_msgs::InteractiveMarker makeGripperMarker(geometry_msgs::PoseStamped pose);

private:

  static visualization_msgs::Marker createGripperMeshMarker(double x, double y, double z, double rx, double ry, double rz, double rw, std::string meshLocation);
};

#endif
