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

#include "perception_yx/camera_process.h"
#include "common/perception_gflags.h"

namespace apollo_perception_standalone {

using Eigen::Affine3d;
using Eigen::Matrix4d;

bool CameraProcess::Init(int cam_idx, std::string &file_path_config, bool log_cam) {
  
  log_cam_ = log_cam;

  int cam_idx_ = cam_idx;
  auto ccm = Singleton<CalibrationConfigManager>::get();
  CameraCalibrationPtr calibrator = ccm->get_camera_calibration(cam_idx_);

  calibrator->get_image_height_width(&image_height_, &image_width_);
  // AWARN << "image height and width: " << image_height_ << " " << image_width_;
  camera_to_car_ = calibrator->get_camera_extrinsics();  // camera to lidar
  intrinsics_ = calibrator->get_camera_intrinsic();

  detector_.reset(new YoloCameraDetector);
  static_cast<YoloCameraDetector *>(detector_.get())->Init(file_path_config);

  converter_.reset(new GeometryCameraConverter);
  static_cast<GeometryCameraConverter *>(converter_.get())->Init(file_path_config);

  tracker_.reset(new CascadedCameraTracker);
  tracker_->Init();

  transformer_.reset(new FlatCameraTransformer);
  transformer_->Init();
  transformer_->SetExtrinsics(camera_to_car_);

  filter_.reset(new ObjectCameraFilter);
  filter_->Init();

  if (!CameraFrameSupplement::state_vars.initialized_) {
    // CameraFrameSupplement::state_vars.process_noise *= 10;
    // CameraFrameSupplement::state_vars.trans_matrix.block(0, 0, 1, 4) << 1.0f,
    //     0.0f, 0.33f, 0.0f;
    // CameraFrameSupplement::state_vars.trans_matrix.block(1, 0, 1, 4) << 0.0f,
    //     1.0f, 0.0f, 0.33f;
    ADEBUG << "state trans matrix in CameraFrameSupplement is \n"
           << CameraFrameSupplement::state_vars.trans_matrix << std::endl;
    CameraFrameSupplement::state_vars.initialized_ = true;
  }

  AINFO << "Init successfully, CameraProcess";

  return true;
}

// void CameraProcess::ImgCallback(const sensor_msgs::Image &message) {
void CameraProcess::ImgCallback(double timestamp,
                                std::shared_ptr<SensorObjects> &sensor_objects,
                                cv::Mat &img) {
  
  // double timestamp = message.header.stamp.toSec();
  // ADEBUG << "CameraProcess ImgCallback: timestamp: ";
  // ADEBUG << std::fixed << std::setprecision(64) << timestamp;
  // AINFO << "camera received image : " << GLOG_TIMESTAMP(timestamp)
  //       << " at time: " << GLOG_TIMESTAMP(TimeUtil::GetCurrentTime());
  // double curr_timestamp = timestamp * 1e9;

  // if (FLAGS_skip_camera_frame && timestamp_ns_ > 0.0) {
  //   if ((curr_timestamp - timestamp_ns_) < (1e9 / FLAGS_camera_hz) &&
  //       curr_timestamp > timestamp_ns_) {
  //     ADEBUG << "CameraProcess Skip frame";
  //     return;
  //   }
  // }
  // timestamp_ns_ = curr_timestamp;

  // AWARN_IF(log_cam_) << "CameraProcess Process Started!";
  PERF_FUNCTION("CameraProcess");
  PERF_BLOCK_START();

  std::vector<std::shared_ptr<VisualObject>> objects;
  cv::Mat mask;

  detector_->Multitask(img, CameraDetectorOptions(), &objects, &mask);
  // cv::Mat mask_color(mask.rows, mask.cols, CV_32FC1);
  // if (FLAGS_use_whole_lane_line) {
  //   std::vector<cv::Mat> masks;
  //   detector_->Lanetask(img, &masks);
  //   mask_color.setTo(cv::Scalar(0));
  //   ln_msk_threshold_ = 0.9;
  //   for (int c = 0; c < num_lines; ++c) {
  //     for (int h = 0; h < masks[c].rows; ++h) {
  //       for (int w = 0; w < masks[c].cols; ++w) {
  //         if (masks[c].at<float>(h, w) >= ln_msk_threshold_) {
  //           mask_color.at<float>(h, w) = static_cast<float>(c);
  //         }
  //       }
  //     }
  //   }
  // } else {
  //   mask.copyTo(mask_color);
  //   ln_msk_threshold_ = 0.5;
  //   for (int h = 0; h < mask_color.rows; ++h) {
  //     for (int w = 0; w < mask_color.cols; ++w) {
  //       if (mask_color.at<float>(h, w) >= ln_msk_threshold_) {
  //         mask_color.at<float>(h, w) = static_cast<float>(5);
  //       }
  //     }
  //   }
  // }
  PERF_BLOCK_END("CameraProcess_detector_");

  converter_->Convert(&objects);
  PERF_BLOCK_END("CameraProcess_converter_");
  // AWARN_IF(log_cam_) << "Convert done...";

  if (FLAGS_use_navigation_mode) {
    // get pandora2vehicle transfrom
    std::shared_ptr<Eigen::Matrix4d> pandora_trans = std::make_shared<Eigen::Matrix4d>();
    if (!GetSensorTrans(timestamp, pandora_trans.get())) {
      AERROR << "failed to get trans at timestamp: " << timestamp;
      return;
    }
    transformer_->Transform(&objects, pandora_trans);
    adjusted_extrinsics_ =
        transformer_->GetAdjustedExtrinsics(&camera_to_car_adj_);
    PERF_BLOCK_END("CameraProcess_transformer_");
  }
  // AWARN_IF(log_cam_) << "Transform done...";

  tracker_->Associate(img, timestamp, &objects);
  PERF_BLOCK_END("CameraProcess_tracker_");
  // AWARN_IF(log_cam_) << "Track done...";

  FilterOptions options;
  if (FLAGS_use_navigation_mode) {  // camera_trans: sensor to vehicle
    options.camera_trans = std::make_shared<Eigen::Matrix4d>();
    // options.camera_trans->setIdentity();
    if (!GetSensorTrans(timestamp, options.camera_trans.get())) {
      AERROR << "failed to get trans at timestamp: " << timestamp;
      return;
    }
  } 
  else {  // camera_trans: sensor to world
    AERROR << "Need to switch on Navigation Mode";
    return;
    // options.camera_trans = std::make_shared<Eigen::Matrix4d>();
    // if (!GetCameraTrans(timestamp, options.camera_trans.get())) {
    //   AERROR << "failed to get trans at timestamp: " << timestamp;
    //   return;
    // }
  }
  camera_to_world_ = *(options.camera_trans);
  // need to create camera options here for filter
  filter_->Filter(timestamp, &objects, options);
  PERF_BLOCK_END("CameraProcess_filter_");
  // AWARN_IF(log_cam_) << "Filter done...";

  auto ccm = Singleton<CalibrationConfigManager>::get();
  auto calibrator = ccm->get_camera_calibration(cam_idx_);
  calibrator->SetCar2CameraExtrinsicsAdj(camera_to_car_adj_,
                                         adjusted_extrinsics_);

  sensor_objects_.reset(new SensorObjects);
  sensor_objects_->timestamp = timestamp;
  VisualObjToSensorObj(objects, sensor_objects_, options);
  // mask_color.copyTo(sensor_objects_->camera_frame_supplement->lane_map);
  // AWARN_IF(log_cam_) << "Visual2Sensor done...";

  *sensor_objects = *sensor_objects_;
  PERF_BLOCK_END("CameraProcess completed");

}

void CameraProcess::MockCameraPolygon(const Eigen::Vector3d &center,
                                 const double length, const double width,
                                 const double theta, PolygonDType *polygon) {
  if (polygon == nullptr) {
    AERROR << "polygon is nullptr";
    return;
  }
  Eigen::Matrix2d rotation;
  rotation << cos(theta), -sin(theta), sin(theta), cos(theta);
  Eigen::Vector2d local_poly(0, 0);
  Eigen::Vector2d world_poly;
  polygon->resize(4);
  local_poly(0) = -0.5 * length;
  local_poly(1) = -0.5 * width;
  world_poly = rotation * local_poly;
  polygon->points[0].x = center(0) + world_poly(0);
  polygon->points[0].y = center(1) + world_poly(1);
  polygon->points[0].z = center(2);
  local_poly(0) = -0.5 * length;
  local_poly(1) = +0.5 * width;
  world_poly = rotation * local_poly;
  polygon->points[1].x = center(0) + world_poly(0);
  polygon->points[1].y = center(1) + world_poly(1);
  polygon->points[1].z = center(2);
  local_poly(0) = +0.5 * length;
  local_poly(1) = +0.5 * width;
  world_poly = rotation * local_poly;
  polygon->points[2].x = center(0) + world_poly(0);
  polygon->points[2].y = center(1) + world_poly(1);
  polygon->points[2].z = center(2);
  local_poly(0) = +0.5 * length;
  local_poly(1) = -0.5 * width;
  world_poly = rotation * local_poly;
  polygon->points[3].x = center(0) + world_poly(0);
  polygon->points[3].y = center(1) + world_poly(1);
  polygon->points[3].z = center(2);

  return;
}

void CameraProcess::VisualObjToSensorObj(
    const std::vector<std::shared_ptr<VisualObject>> &objects,
    std::shared_ptr<SensorObjects> &sensor_objects_, FilterOptions options) {
  sensor_objects_->sensor_type = SensorType::PANDORA_CAMERA;
  sensor_objects_->sensor_id = device_id_;
  sensor_objects_->seq_num = seq_num_;
  if (FLAGS_use_navigation_mode) {
    sensor_objects_->sensor2world_pose_static = camera_to_car_;
    sensor_objects_->sensor2world_pose = camera_to_car_adj_;
  } else {
    sensor_objects_->sensor2world_pose_static = *(options.camera_trans);
    sensor_objects_->sensor2world_pose = *(options.camera_trans);
    AINFO << "camera process sensor2world pose is "
          << sensor_objects_->sensor2world_pose;
  }
  (sensor_objects_->camera_frame_supplement).reset(new CameraFrameSupplement);

  for (auto vobj : objects) {
    std::unique_ptr<Object> obj(new Object());

    obj->id = vobj->id;
    obj->score = vobj->score;
    obj->direction = vobj->direction.cast<double>();
    obj->theta = vobj->theta;
    obj->center = vobj->center.cast<double>();
    obj->length = vobj->length;
    obj->width = vobj->width;
    obj->height = vobj->height;
    // based on MockRadarPolygon
    MockCameraPolygon(obj->center, obj->length, obj->width,
                      obj->theta, &(obj->polygon));
    obj->type = vobj->type;
    obj->track_id = vobj->track_id;
    obj->tracking_time = vobj->track_age;
    obj->latest_tracked_time = vobj->last_track_timestamp;
    obj->velocity = vobj->velocity.cast<double>();
    obj->anchor_point = obj->center;
    obj->state_uncertainty = vobj->state_uncertainty;

    (obj->camera_supplement).reset(new CameraSupplement());
    obj->camera_supplement->upper_left = vobj->upper_left.cast<double>();
    obj->camera_supplement->lower_right = vobj->lower_right.cast<double>();
    obj->camera_supplement->alpha = vobj->alpha;
    obj->camera_supplement->pts8 = vobj->pts8;
    (sensor_objects_->objects).emplace_back(obj.release());
  }
}

bool CameraProcess::GetSensorTrans(const double query_time, Eigen::Matrix4d* trans) {
    if (!trans) {
      AERROR << "failed to get trans, the trans ptr can not be nullptr";
      return false;
    }

    ros::Time query_stamp(query_time);
    // const double kTf2BuffSize = FLAGS_tf2_buff_in_ms / 1000.0;
    std::string err_msg;
    if (!tf2_listener_.canTransform(FLAGS_camera_tf2_frame_id,
                                 FLAGS_camera_tf2_child_frame_id,
                                 query_stamp,
                                //  ros::Time(0),
                                 &err_msg)) {
      AERROR << "Cannot transform frame: " << FLAGS_camera_tf2_frame_id
             << " to frame " << FLAGS_camera_tf2_child_frame_id
             << " , err: " << err_msg
             << ". Frames: " << tf2_listener_.allFramesAsString();
      return false;
    }

    tf::StampedTransform stamped_transform;
    // geometry_msgs::TransformStamped transform_stamped;
    try {
      tf2_listener_.lookupTransform(FLAGS_camera_tf2_frame_id, 
        FLAGS_camera_tf2_child_frame_id, query_stamp, stamped_transform);
    } catch (tf2::TransformException& ex) {
      AERROR << "Exception: " << ex.what();
      return false;
    }
    Affine3d affine_3d;
    tf::transformTFToEigen(stamped_transform, affine_3d);
    *trans = affine_3d.matrix();

    // AWARN_IF(log_cam_) << "get " << FLAGS_camera_tf2_frame_id << " to "
    //        << FLAGS_camera_tf2_child_frame_id << " trans: " << *trans;
    return true;
}

}  // namespace apollo
