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

#ifndef _APOLLO_PERCEPTION_STANDALONE_RADAR_MODEST_MODEST_RADAR_DETECTOR_H_
#define _APOLLO_PERCEPTION_STANDALONE_RADAR_MODEST_MODEST_RADAR_DETECTOR_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <memory>
#include <string>
#include <vector>

#include "radar/proto/modest_radar_detector_config.pb.h"

#include "radar/interface/base_radar_detector.h"
#include "radar/modest/object_builder.h"
#include "radar/modest/radar_track_manager.h"

namespace apollo_perception_standalone {

class ModestRadarDetector : public BaseRadarDetector {
 public:
  ModestRadarDetector() : BaseRadarDetector() {}
  //~ModestRadarDetector() override = default;

  bool Init(std::string &folder_path_config) override;

  bool Init(std::string &folder_path_config, bool log_radar);

  // @brief: Radar raw obstacles -> objects.
  // @param [in]: raw obstacles from radar driver.
  // @param [in]: roi map polygons, using world frame.
  // @param [in]: options.
  // @param [out]: transformed objects.
  // return true if detect successfully, otherwise return false
  bool Detect(const ContiRadar &raw_obstacles,
              const std::vector<PolygonDType> &map_polygons,
              const RadarDetectorOptions &options,
              std::vector<std::shared_ptr<Object>> *objects) override;

  // @brief: collect radar result
  // @param [out]: radar objects
  // @return collection state
  bool CollectRadarResult(std::vector<std::shared_ptr<Object>> *objects);

  bool GetSensorTrans(const double query_time, Eigen::Matrix4d* trans);

  std::string name() const override { return "ModestRadarDetector"; }

 private:
  void RoiFilter(const std::vector<PolygonDType> &map_polygons,
                 std::vector<std::shared_ptr<Object>> *filter_objects);

  // logging 
  bool log_radar_ = false;

  tf::TransformListener tf2_listener_;

  // for unit test
  bool result_init_ = true;
  bool result_detect_ = true;

  ContiParams conti_params_;
  ObjectBuilder object_builder_;
  boost::shared_ptr<RadarTrackManager> radar_tracker_;

  modest_radar_detector_config::ModelConfigs config_;

};

}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_RADAR_MODEST_MODEST_RADAR_DETECTOR_H_
