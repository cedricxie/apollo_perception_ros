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

#ifndef _APOLLO_PERCEPTION_STANDALONE_COMMON_TYPES_H_
#define _APOLLO_PERCEPTION_STANDALONE_COMMON_TYPES_H_

#include <string>

#include "common/pcl_types.h"

namespace apollo_perception_standalone {

enum class ObjectType {
  UNKNOWN = 0,
  UNKNOWN_MOVABLE = 1,
  UNKNOWN_UNMOVABLE = 2,
  PEDESTRIAN = 3,
  BICYCLE = 4,
  VEHICLE = 5,
  MAX_OBJECT_TYPE = 6,
};

enum InternalObjectType {
  INT_BACKGROUND = 0,
  INT_SMALLMOT = 1,
  INT_PEDESTRIAN = 2,
  INT_NONMOT = 3,
  INT_BIGMOT = 4,
  INT_UNKNOWN = 5,
  INT_MAX_OBJECT_TYPE = 6,
};

enum class SensorType {
  VELODYNE_64 = 0,
  VELODYNE_16 = 1,
  RADAR = 2,
  CAMERA = 3,
  ULTRASONIC = 4,
  PANDORA_HESAI40 = 5,
  PANDORA_CAMERA = 6,
  FUJITSU_ESR = 7,
  BOSCH_MRR = 8,
  FUSION = 9,
  IBEO = 10,
  DELPHI_ESR = 11,
  DELPHI_SRR = 12,
  UNKNOWN_SENSOR_TYPE = 13,
};

typedef apollo_perception_standalone::pcl_util::PointCloud PolygonType;
typedef apollo_perception_standalone::pcl_util::PointDCloud PolygonDType;

using SeqId = uint32_t;

std::string GetObjectName(const ObjectType& obj_type);

std::string GetSensorType(SensorType sensor_type);

bool is_lidar(SensorType sensor_type);
bool is_radar(SensorType sensor_type);
bool is_camera(SensorType sensor_type);

}  // namespace perception

#endif  // MODULES_PERCEPTION_OBSTACLE_BASE_TYPES_H_
