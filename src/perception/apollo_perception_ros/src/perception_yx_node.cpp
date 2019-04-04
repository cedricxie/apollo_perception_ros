/**
 * @file perception_yx_node.cpp
 * @author Yuesong Xie (yxie@hra.com)
 * @brief A ROS node that runs apollo perception algorithm
 * @version 0.1
 * @date 2018-11-16
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#include <ros/ros.h>
#include <perception_yx/perception_yx.h>

/** Main entry point. */
int main(int argc, char **argv)
{
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
  
  ros::init(argc, argv, "perception_yx_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create perception class
  perception_yx::PerceptionYX yx(node, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
