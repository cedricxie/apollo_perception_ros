/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

// The base class of Id association for camera objects between frames

#ifndef _APOLLO_PERCEPTION_STANDALONE_CAMERA_INTERFACE_BASE_CAMERA_TRACKER_H_
#define _APOLLO_PERCEPTION_STANDALONE_CAMERA_INTERFACE_BASE_CAMERA_TRACKER_H_

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "opencv2/opencv.hpp"

#include "camera/common/visual_object.h"

namespace apollo_perception_standalone {

class BaseCameraTracker {
 public:
  BaseCameraTracker() {}
  virtual ~BaseCameraTracker() {}

  virtual bool Init() = 0;

  // @brief: Assign global track id for camera objects (ID association)
  // @param [in/out]: object lists, added tracking related information
  virtual bool Associate(
      const cv::Mat& img, const double timestamp,
      std::vector<std::shared_ptr<VisualObject>>* objects) = 0;

  virtual std::string Name() const = 0;

};

}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_CAMERA_TRACKER_H_
