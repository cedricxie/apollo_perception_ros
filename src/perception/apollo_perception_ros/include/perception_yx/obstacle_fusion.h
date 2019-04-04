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

#ifndef _PERCEPTION_YX_OBSTACLE_PERCEPTION_H_
#define _PERCEPTION_YX_OBSTACLE_PERCEPTION_H_

#include <memory>
#include <vector>

#include "common/object.h"
#include "common/sensor_raw_frame.h"

#include "fusion/interface/base_fusion.h"

namespace apollo_perception_standalone {

enum ObstacleShowType {
  SHOW_LIDAR = 0,
  SHOW_RADAR = 1,
  SHOW_FUSED = 2,
  MAX_SHOW_TYPE
};

class ObstacleFusion {
 public:
  /**
   * @brief Construct
   */
  ObstacleFusion();

  /**
   * @brief Destruct
   */
  ~ObstacleFusion();

  /**
   * @brief Initialize configuration
   * @return True if initialize successfully, false otherwise
   */
  bool Init(std::string &folder_path_config, bool log_fusion);

  /**
   * @brief The main process to detect, recognize and track objects
   * based on different kinds of sensor data.
   * @param frame Sensor data of one single frame
   * @param out_objects The obstacle perception results
   * @return True if process successfully, false otherwise
   */
  bool Process(std::shared_ptr<apollo_perception_standalone::SensorObjects> &in_sensor_objects,
               std::vector<std::shared_ptr<Object>>* out_objects);

 private:

  /// obstacle fusion
  std::unique_ptr<BaseFusion> fusion_;

  bool lidar_pose_inited_;
  bool log_fusion_ = false;

};  // class ObstaclePerception

}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBOARD_OBSTACLE_PERCEPTION_H_
