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

#ifndef _PERCEPTION_YX_CAMERA_PROCESS_H_
#define _PERCEPTION_YX_CAMERA_PROCESS_H_

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "yaml-cpp/yaml.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include "util/log.h"
#include "util/time_util.h"
#include "util/timer.h"
#include "util/singleton.h"
#include "cuda_util/util.h"
#include "common/object.h"
#include "common/types.h"
#include "common/perception_gflags.h"

#include "config_manager/calibration_config_manager.h"

#include "camera/dummy/dummy_algorithms.h"
#include "camera/converter/geometry_camera_converter.h"
#include "camera/detector/yolo_camera_detector/yolo_camera_detector.h"
#include "camera/filter/object_camera_filter.h"
#include "camera/interface/base_camera_converter.h"
#include "camera/interface/base_camera_detector.h"
#include "camera/interface/base_camera_filter.h"
#include "camera/interface/base_camera_tracker.h"
#include "camera/interface/base_camera_transformer.h"
#include "camera/tracker/cascaded_camera_tracker.h"
#include "camera/transformer/flat_camera_transformer.h"

#include "proto/perception_obstacle.pb.h"
//#include "modules/perception/traffic_light/util/color_space.h"

namespace apollo_perception_standalone {

class CameraProcess {
 public:
  CameraProcess() = default;
  ~CameraProcess() = default;

  bool Init(int cam_idx, std::string &file_path_config, bool log_cam);

//   void ImgCallback(const sensor_msgs::Image& message);
  void ImgCallback(double timestamp,
                   std::shared_ptr<SensorObjects> &sensor_objects,
                   cv::Mat &image);

//   bool MessageToMat(const sensor_msgs::Image& msg, cv::Mat* img);
//   bool MatToMessage(const cv::Mat& img, sensor_msgs::Image* msg);

  void VisualObjToSensorObj(
      const std::vector<std::shared_ptr<VisualObject>>& objects,
      std::shared_ptr<SensorObjects> &sensor_objects_, FilterOptions options);

  void MockCameraPolygon(const Eigen::Vector3d &center,
                                 const double length, const double width,
                                 const double theta, PolygonDType *polygon);

  bool GetSensorTrans(const double query_time, Eigen::Matrix4d* trans);

  // General
  int cam_idx_;
  std::string device_id_ = "camera";
  SeqId seq_num_ = 0;
  double timestamp_ns_ = 0.0;
  bool log_cam_ = false;

  tf::TransformListener tf2_listener_;

  // Sensor Objects
  std::shared_ptr<SensorObjects> sensor_objects_;

  // Calibration
  // int32_t image_height_ = 720;
  // int32_t image_width_ = 1280;
  int32_t image_height_ = 1080;
  int32_t image_width_ = 1920;  // default 1080P
  Eigen::Matrix4d camera_to_car_;
  Eigen::Matrix<double, 3, 4> intrinsics_;

  // Dynamic calibration based on objects
  // Always available, but retreat to static one if flag is false
  bool adjusted_extrinsics_ = false;
  Eigen::Matrix4d camera_to_car_adj_;
  Eigen::Matrix4d camera_to_world_;

  // Publish to Perception Protobuf and ROS topic
  bool pb_obj_ = false;  // Objects
  bool pb_ln_msk_ = false;  // Lane marking mask
  float ln_msk_threshold_ = 0.95f;
  const int num_lines = 13;

  // Modules
  std::unique_ptr<BaseCameraDetector> detector_;
  std::unique_ptr<BaseCameraConverter> converter_;
  std::unique_ptr<BaseCameraTracker> tracker_;
  std::unique_ptr<FlatCameraTransformer> transformer_;
  std::unique_ptr<BaseCameraFilter> filter_;

  private:

};

}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBORAD_CAMERA_PROCESS_SUBNODE_H_
