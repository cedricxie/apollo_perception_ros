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

#ifndef _APOLLO_PERCEPTION_STANDALONE_FUSION_INTERFACE_BASE_FUSION_H_
#define _APOLLO_PERCEPTION_STANDALONE_FUSION_INTERFACE_BASE_FUSION_H_
/** @example
 * <pre>
 * class MyObjectFusion : public BaseFusion {
 * public:
 *     MyObjectFusion() : BaseFusion() {}
 *     virtual ~MyObjectFusion() {}
 *
 *     virtual bool Init() override {
 *         // Do something.
 *         return true;
 *     }
 *
 *     virtual bool Fuse(
 *              const std::vector<SensorObjects>& multi_sensor_objects,
 *              std::vector<std::shared_ptr<Object>>* fused_objects) override {
 *
 *          // Do something.
 *          return true;
 *      }
 *
 *      virtual std::string name() const override {
 *          return "MyObjectFusion";
 *      } //
 * };
 *
 * // Register plugin.
 * REGISTER_FUSION(MyObjectFusion);
 *
 *
 * // USING CODE:
 *
 * BaseFusion* fusion =
 *  BaseFusionRegisterer::get_instance_by_name("MyObjectFusion");
 * using fusion to do somethings.
 * </pre>
 **/
#include <memory>
#include <string>
#include <vector>

#include "common/pcl_types.h"
#include "common/object.h"
#include "common/types.h"

namespace apollo_perception_standalone {

class FusionOptions {
 public:
  std::vector<double> fused_frame_ts;
  std::vector<std::string> fused_frame_device_id;
  bool fused = false;
};

class BaseFusion {
 public:
  BaseFusion() = default;
  virtual ~BaseFusion() = default;
  virtual bool Init() = 0;

  // @brief: fuse objects from multi sensors(64-lidar, 16-lidar, radar...)
  // @param [in]: multi sensor objects.
  // @param [out]: fused objects.
  // @return true if fuse successfully, otherwise return false
  virtual bool Fuse(const std::vector<SensorObjects> &multi_sensor_objects,
                    std::vector<std::shared_ptr<Object>> *fused_objects,
                    FusionOptions *options) = 0;
  virtual std::string name() const = 0;

};

}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_FUSION_INTERFACE_BASE_FUSION_H_
