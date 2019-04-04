/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef _APOLLO_PERCEPTION_STANDALONE_LIDAR_ROI_FILTER_H_
#define _APOLLO_PERCEPTION_STANDALONE_LIDAR_ROI_FILTER_H_

#include <memory>
#include <string>
#include <vector>

#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "common/object.h"
#include "common/pcl_types.h"

#include "util/log.h"

#include "lidar/proto/roi_filter_config.pb.h"

namespace apollo_perception_standalone {

struct ROIFilterOptions {
  ROIFilterOptions() = default;
};

class ROIFilter {
 public:
  ROIFilter() {}
  ~ROIFilter() {}

  bool Init(std::string &file_path_roi_filter_config);

  std::string name() const { return "ROIFilter"; }

  bool Filter(const ROIFilterOptions &roi_filter_options,
              const pcl_util::VPointCloudPtr &cloud_ptr, 
              pcl_util::VPointCloudPtr &filtered_cloud_all_ptr,
              pcl_util::PointIndicesPtr &filtered_cloud_indices_ptr);

 protected:
  void FilterROI(const ROIFilterOptions& roi_filter_options,
                 const pcl_util::VPointCloudPtr &cloud_ptr);

 private:
  roi_filter_config::ModelConfigs config_;

  // Parameters that define XY RoI
  double roi_x_min_;
  double roi_x_max_;
  double roi_y_min_;
  double roi_y_max_;
  // Parameters that define Groud Plane Removal
  double dist_gplane_;
  double roi_z_min_;
  double roi_z_max_;

  pcl_util::VPointCloudPtr filtered_cloud_all_ptr_; // point cloud after X and Y filtering
  pcl_util::PointIndicesPtr filtered_cloud_indices_ptr_; // indices for non-ground points
};

}  // namespace apollo
#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_OBJECT_FILTER_LOW_OBJECT_FILTER_H_
