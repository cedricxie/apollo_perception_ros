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
#include "perception_yx/obstacle_fusion.h"

#include <string>

#include "util/log.h"

#include "common/perception_gflags.h"

#include "fusion/probabilistic_fusion/probabilistic_fusion.h"
#include "fusion/async_fusion/async_fusion.h"
#include "radar/modest/modest_radar_detector.h"

namespace apollo_perception_standalone {

ObstacleShowType GetObstacleShowType(const std::string& show_type_string) {
  if (show_type_string == "lidar") {
    return SHOW_LIDAR;
  } else if (show_type_string == "radar") {
    return SHOW_RADAR;
  } else if (show_type_string == "fused") {
    return SHOW_FUSED;
  } else {
    return MAX_SHOW_TYPE;
  }
}

ObstacleFusion::ObstacleFusion() : lidar_pose_inited_(false) {}

ObstacleFusion::~ObstacleFusion() {}

bool ObstacleFusion::Init(std::string &folder_path_config, bool log_fusion) {
  
  log_fusion_ = log_fusion;
  
  /// initialize fusion
  if (FLAGS_async_fusion) {
    fusion_.reset(new AsyncFusion);
  } else {
    fusion_.reset(new ProbabilisticFusion);
  }
  if (fusion_ == nullptr) {
    AERROR << "Failed to get fusion instance: " ;
    // << FLAGS_onboard_fusion;
    return false;
  }
  if (FLAGS_async_fusion) {
    if (!static_cast<AsyncFusion *>(fusion_.get())->Init(log_fusion_, folder_path_config)) {
      AERROR << "Failed to init fusion:" ;
      return false;
    }
  } else {
    if (!static_cast<ProbabilisticFusion *>(fusion_.get())->Init(log_fusion_, folder_path_config)) {
      AERROR << "Failed to init fusion:" ;
      return false;
    }
  }
  
  // AWARN_IF(log_fusion_) << "Fusion: obstacle fusion initialized successfully.";
  return true;
}

bool ObstacleFusion::Process(
    std::shared_ptr<apollo_perception_standalone::SensorObjects> &in_sensor_objects, 
    std::vector<std::shared_ptr<Object>>* out_objects) {
  if (in_sensor_objects == nullptr || out_objects == nullptr) {
    return false;
  }

  if (is_lidar(in_sensor_objects->sensor_type)) {
    /// lidar obstacle
    // AWARN_IF(log_fusion_) << "Fusion: lidar objects size: " << in_sensor_objects->objects.size();
    lidar_pose_inited_ = true;
  } else if (is_radar(in_sensor_objects->sensor_type)) {
    /// radar obstacle 
    // AWARN_IF(log_fusion_) << "Fusion: radar objects size: " << in_sensor_objects->objects.size();
  } else if (is_camera(in_sensor_objects->sensor_type)) {
    /// camera obstacle 
    // AWARN_IF(log_fusion_) << "Fusion: camera objects size: " << in_sensor_objects->objects.size();
  } else {
    AERROR << "Unknown sensor type";
    return false;
  }

  /// fusion
  std::vector<SensorObjects> multi_sensor_objs;
  multi_sensor_objs.push_back(*in_sensor_objects);
  std::vector<std::shared_ptr<Object>> fused_objects;
  FusionOptions options;

  if (!fusion_->Fuse(multi_sensor_objs, &fused_objects, &options)) {
    AERROR << "Failed to fusion";
    return false;
  }
  *out_objects = fused_objects;
  AWARN_IF(log_fusion_) << "fused objects size: " << fused_objects.size();

  return true;
}

}  // namespace apollo
