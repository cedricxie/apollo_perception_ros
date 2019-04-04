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

#ifndef _APOLLO_PERCEPTION_STANDALONE_LIDAR_TRACKER_FEATURE_DESCRIPTOR_H_
#define _APOLLO_PERCEPTION_STANDALONE_LIDAR_TRACKER_FEATURE_DESCRIPTOR_H_

#include <algorithm>
#include <vector>

#include "common/pcl_types.h"

namespace apollo_perception_standalone {

class FeatureDescriptor {
 public:
  // @brief initialize feature descriptor
  // @param[IN] cloud: given cloud for feature extraction
  // @return nothing
  FeatureDescriptor(
      pcl_util::PointCloudPtr cloud) {
    cloud_ = cloud;
  }
  ~FeatureDescriptor() {}

  // @brief compute histogram feature of given cloud
  // @param[IN] bin_size: bin size of histogram
  // @param[OUT] feature: histogram feature of given cloud
  // @return nothing
  void ComputeHistogram(const int bin_size, std::vector<float>* feature);

 private:
  void GetMinMaxCenter();
  pcl_util::PointCloudPtr cloud_;
  pcl_util::Point min_pt_;
  pcl_util::Point max_pt_;
  pcl_util::Point center_pt_;
};  // class FeatureDescriptor

}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_CLASSIFIER_FEATURE_DESCRIPTOR_H_
