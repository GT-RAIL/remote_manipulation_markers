# remote_manipulation_markers [![Build Status](https://api.travis-ci.org/GT-RAIL/remote_manipulation_markers.png)](https://travis-ci.org/GT-RAIL/remote_manipulation_markers)
A set of interactive markers for various methods of remote teleoperation manipulation of 6-DOF robot end-effectors.

## Quick Start Guide
This package comes with a demo for the Point-and-Click interaction mode, using RAIL packages for grasp calculation.  Follow the following steps to run the demo.

  1. Clone the following RAIL packages into a ROS catkin workspace :
     * remote_manipulation_markers (https://github.com/GT-RAIL/remote_manipulation_markers) - backend for the point-and-click interface
     * rail_agile (https://github.com/gt-rail/rail_agile) - grasp sampling
     * rail_grasp_calculation (https://github.com/GT-RAIL/rail_grasp_calculation) - grasp ranking
     * rail_manipulation_msgs (https://github.com/GT-RAIL/rail_manipulation_msgs) - supporting ROS message types
  1. Build your ROS workspace
  1. Launch a depth camera
     * The demo assumes an asus xtion camera, but can be adjusted with launch file parameters
     * Start the asus xtion with:  ```$ roslaunch openni_launch openni.launch```
  1. Launch the point-and-click demo  
     * The complete back-end for the demo can be started with the launch file: ```$ roslaunch remote_manipulation_markers point_and_click_demo.launch```
     * The following parameters may be useful to adjust for your system:
        * If using a different depth camera, change `cloud_topic` to your camera's point cloud ROS topic
        * Grasp sampling parameters are configured for the Robotiq-85 2-finger gripper.  If you're using a different gripper, adjust the parameters under `<!-- gripper parameters for grasp sampling; note: defaults are measurements of the robotiq-85 gripper -->` in the launch file.  These parameters correspond to parameters of [agile_grasp](http://wiki.ros.org/agile_grasp)
        * Other grasp sampling behavior can be adjusted with the parameters under `<!-- grasp sampling and ranking parameters -->`, which correspond to parameters of the [rail_grasp_calculation](https://github.com/GT-RAIL/rail_grasp_calculation) package
        * By default the demo assumes you're performing tabletop manipulation.  If this is not true, set `remove_table` to false
  1. Visualize the interface with rviz  
     * Launch rviz: `$ rosrun rviz rviz`  
     * Add the following display types to rviz
        * `InteractiveMarkers`, set `Update Topic` to `/clickable_point_cloud/update`. This will give you a point cloud to click as input to the Point-and-Click approach.
        * `InteractiveMarkers`, set `Update Topic` to `/grasp_selector/update`.  This will display the grasp to execute.
        * (optional for debugging) `PoseArray`, set `Topic` to `/point_and_click_demo/sampled_grasps`.  This will display poses for the full set of calculated grasps.
  1. To calculate grasps in a local area, click once on a point in the point cloud.  Grasp sampling ranking may take some time depending on the grasp parameters set in the launch file, but once finished a purple gripper marker will appear at the top ranked grasp.
  1. To scroll through grasps, use the ROS service `/point_and_click/cycle_grasps`, callable from the command line with `$ rosservice call /point_and_click/cycle_grasps "forward: true"`
  1. This demo does not assume a hardware or robot control implementation for grasp execution, but it does provide an actionlib interface.  To execute a grasp, use the action client `/point_and_click/execute_grasp`, which assumes you have implemented the grasp execution action server for your hardware.
  
To create your own Point-and-Click interface, use the `/clickable_point_cloud` and `/grasp_selector` interactive marker servers, the `/point_and_click/cycle_grasps` service, and the `/point_and_click/execute_grasp` action client to your preferred visualization environment, such as `rviz` panels or [Robot Web Tools](http://robotwebtools.org/) visualizers.

## Documentation
Full documentation is available on the ROS wiki [here](http://wiki.ros.org/remote_manipulation_markers).
