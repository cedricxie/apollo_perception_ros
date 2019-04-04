/* -*- mode: C++ -*- */
/*  Copyright (C) 2010 UT-Austin & Austin Robot Technology,
 *  David Claridge, Michael Quinlan
 * 
 *  License: Modified BSD Software License 
 */


#ifndef _PERCEPTION_YX_LIDAR_PROCESS_H_
#define _PERCEPTION_YX_LIDAR_PROCESS_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h> // convert between ROS and PCL pointcloud
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <cstddef>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "Eigen/Core"
#include <pcl/point_types.h>

#include "common/object.h"
#include "common/pcl_types.h"
#include "common/perception_gflags.h"

#include "util/log.h"
#include "util/util.h"
#include "util/file.h"
#include "util/pose_util.h"
#include "util/geometry_util.h"

#include "lidar/roi_filter/roi_filter.h"
#include "lidar/segmentation/cnnseg/cnn_segmentation.h"
#include "lidar/object_builder/min_box.h"
#include "lidar/object_filter/low_object_filter.h"
#include "lidar/tracker/hm_tracker.h"

namespace apollo_perception_standalone {
class LidarProcess {
 public:
  LidarProcess() = default;
  ~LidarProcess() = default;

  bool Init(std::string &path_config_folder, bool log_lidar);
  bool IsInit() { return inited_; }

  bool Process(const pcl_util::VPointCloudPtr &cloud_ptr,
               pcl_util::VPointCloudPtr &filtered_cloud_ptr,
               pcl_util::VPointCloudPtr &filtered_cloud_ground_ptr);
  bool Process(const double timestamp, 
               const pcl_util::VPointCloudPtr &cloud_ptr,
               std::shared_ptr<Eigen::Matrix4d> lidar_trans,
               pcl_util::VPointCloudPtr &filtered_cloud_ptr,
               pcl_util::VPointCloudPtr &filtered_cloud_ground_ptr);

  bool GetSensorTrans(const double query_time, Eigen::Matrix4d* trans);

  std::vector<std::shared_ptr<Object>> GetObjects() { return objects_; }

  void GeneratePbMsg(PerceptionObstacles* obstacles);

 private:
  void TransPointCloudToPCL(const sensor_msgs::PointCloud2& in_msg,
                            pcl_util::PointCloudPtr* out_cloud);

  bool inited_ = false;
  bool log_lidar_ = false;
  double timestamp_ = 0.0;  

  tf::TransformListener tf2_listener_;

  std::vector<std::shared_ptr<apollo_perception_standalone::Object>> objects_; // tracked objects
  
  std::unique_ptr<apollo_perception_standalone::ROIFilter> roi_filter_;
  std::unique_ptr<apollo_perception_standalone::CNNSegmentation> segmentor_;
  std::unique_ptr<apollo_perception_standalone::LowObjectFilter> object_filter_;
  std::unique_ptr<apollo_perception_standalone::MinBoxObjectBuilder> object_builder_;
  std::unique_ptr<apollo_perception_standalone::HmObjectTracker> tracker_;
};

}

#endif