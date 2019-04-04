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

#include "radar/modest/modest_radar_detector.h"

#include "util/file.h"
#include "common/perception_gflags.h"

#include "radar/modest/conti_radar_util.h"
#include "radar/modest/object_builder.h"
#include "radar/modest/radar_util.h"

namespace apollo_perception_standalone {

using apollo_perception_standalone::util::GetProtoFromFile;

//ModestRadarDetector::~ModestRadarDetector() {}

bool ModestRadarDetector::Init(std::string &folder_path_config) {

  std::string file_path_modest_radar_detector_config = folder_path_config 
            + "config/modest_radar_detector_config.pb.txt";
  if (!GetProtoFromFile(file_path_modest_radar_detector_config, &config_)) {
    AERROR << "failed to read in radar config at: " << file_path_modest_radar_detector_config;
    return false;
  }

  if (FLAGS_use_navigation_mode) {
    config_.set_use_had_map(false);
  }
  if (!FLAGS_use_navigation_mode && !config_.has_use_had_map()) {
    AERROR << "use_had_map not found.";
    return false;
  }

  RadarTrack::SetTrackedTimesThreshold(config_.delay_frames());
  object_builder_.SetDelayFrame(config_.delay_frames());
  object_builder_.SetUseFpFilter(config_.use_fp_filter());

  conti_params_.probexist_vehicle = config_.probexist_vehicle();
  conti_params_.probexist_pedestrian = config_.probexist_pedestrian();
  conti_params_.probexist_bicycle = config_.probexist_bicycle();
  conti_params_.probexist_unknown = config_.probexist_unknown();
  conti_params_.lo_vel_rms_vehicle = config_.lo_vel_rms_vehicle();
  conti_params_.la_vel_rms_vehicle = config_.la_vel_rms_vehicle();
  conti_params_.lo_dist_rms_vehicle = config_.lo_dist_rms_vehicle();
  conti_params_.la_dist_rms_vehicle = config_.la_dist_rms_vehicle();
  conti_params_.lo_vel_rms_pedestrian = config_.lo_vel_rms_pedestrian();
  conti_params_.la_vel_rms_pedestrian = config_.la_vel_rms_pedestrian();
  conti_params_.lo_dist_rms_pedestrian = config_.lo_dist_rms_pedestrian();
  conti_params_.la_dist_rms_pedestrian = config_.la_dist_rms_pedestrian();
  conti_params_.lo_vel_rms_bicycle = config_.lo_vel_rms_bicycle();
  conti_params_.la_vel_rms_bicycle = config_.la_vel_rms_bicycle();
  conti_params_.lo_dist_rms_bicycle = config_.lo_dist_rms_bicycle();
  conti_params_.la_dist_rms_bicycle = config_.la_dist_rms_bicycle();
  conti_params_.lo_vel_rms_unknown = config_.lo_vel_rms_unknown();
  conti_params_.la_vel_rms_unknown = config_.la_vel_rms_unknown();
  conti_params_.lo_dist_rms_unknown = config_.lo_dist_rms_unknown();
  conti_params_.la_dist_rms_unknown = config_.la_dist_rms_unknown();

  object_builder_.SetContiParams(conti_params_);
  radar_tracker_.reset(new RadarTrackManager(log_radar_));

  // initialize RadarFrameSupplement for fusion later
  if (!RadarFrameSupplement::state_vars.initialized_) {
    // RadarFrameSupplement::state_vars.process_noise(0, 0) *= 10;
    // RadarFrameSupplement::state_vars.process_noise(1, 1) *= 10;
    // RadarFrameSupplement::state_vars.process_noise(2, 2) *= 10;
    // RadarFrameSupplement::state_vars.process_noise(3, 3) *= 10;
    // RadarFrameSupplement::state_vars.trans_matrix.block(0, 0, 1, 4) << 1.0f,
    //     0.0f, 0.33f, 0.0f;
    // RadarFrameSupplement::state_vars.trans_matrix.block(1, 0, 1, 4) << 0.0f,
    //     1.0f, 0.0f, 0.33f;
    ADEBUG << "state trans matrix in RadarFrameSupplement is \n"
           << RadarFrameSupplement::state_vars.trans_matrix << std::endl;
    RadarFrameSupplement::state_vars.initialized_ = true;
  }

  AINFO << "Initialized the modest radar detector";
  return true;
}

bool ModestRadarDetector::Init(std::string &folder_path_config, bool log_radar) {
  // log setting
  log_radar_ = log_radar;

  std::string file_path_modest_radar_detector_config = folder_path_config 
            + "/config/modest_radar_detector_config.pb.txt";
  if (!GetProtoFromFile(file_path_modest_radar_detector_config, &config_)) {
    AERROR << "failed to read in radar config at: " << file_path_modest_radar_detector_config;
    return false;
  }

  if (FLAGS_use_navigation_mode) {
    config_.set_use_had_map(false);
  }
  if (!FLAGS_use_navigation_mode && !config_.has_use_had_map()) {
    AERROR << "use_had_map not found.";
    return false;
  }

  RadarTrack::SetTrackedTimesThreshold(config_.delay_frames());
  object_builder_.SetDelayFrame(config_.delay_frames());
  object_builder_.SetUseFpFilter(config_.use_fp_filter());

  conti_params_.probexist_vehicle = config_.probexist_vehicle();
  conti_params_.probexist_pedestrian = config_.probexist_pedestrian();
  conti_params_.probexist_bicycle = config_.probexist_bicycle();
  conti_params_.probexist_unknown = config_.probexist_unknown();
  conti_params_.lo_vel_rms_vehicle = config_.lo_vel_rms_vehicle();
  conti_params_.la_vel_rms_vehicle = config_.la_vel_rms_vehicle();
  conti_params_.lo_dist_rms_vehicle = config_.lo_dist_rms_vehicle();
  conti_params_.la_dist_rms_vehicle = config_.la_dist_rms_vehicle();
  conti_params_.lo_vel_rms_pedestrian = config_.lo_vel_rms_pedestrian();
  conti_params_.la_vel_rms_pedestrian = config_.la_vel_rms_pedestrian();
  conti_params_.lo_dist_rms_pedestrian = config_.lo_dist_rms_pedestrian();
  conti_params_.la_dist_rms_pedestrian = config_.la_dist_rms_pedestrian();
  conti_params_.lo_vel_rms_bicycle = config_.lo_vel_rms_bicycle();
  conti_params_.la_vel_rms_bicycle = config_.la_vel_rms_bicycle();
  conti_params_.lo_dist_rms_bicycle = config_.lo_dist_rms_bicycle();
  conti_params_.la_dist_rms_bicycle = config_.la_dist_rms_bicycle();
  conti_params_.lo_vel_rms_unknown = config_.lo_vel_rms_unknown();
  conti_params_.la_vel_rms_unknown = config_.la_vel_rms_unknown();
  conti_params_.lo_dist_rms_unknown = config_.lo_dist_rms_unknown();
  conti_params_.la_dist_rms_unknown = config_.la_dist_rms_unknown();

  object_builder_.SetContiParams(conti_params_);
  radar_tracker_.reset(new RadarTrackManager(log_radar_));

  // initialize RadarFrameSupplement for fusion later
  if (!RadarFrameSupplement::state_vars.initialized_) {
    // RadarFrameSupplement::state_vars.process_noise(0, 0) *= 10;
    // RadarFrameSupplement::state_vars.process_noise(1, 1) *= 10;
    // RadarFrameSupplement::state_vars.process_noise(2, 2) *= 10;
    // RadarFrameSupplement::state_vars.process_noise(3, 3) *= 10;
    // RadarFrameSupplement::state_vars.trans_matrix.block(0, 0, 1, 4) << 1.0f,
    //     0.0f, 0.33f, 0.0f;
    // RadarFrameSupplement::state_vars.trans_matrix.block(1, 0, 1, 4) << 0.0f,
    //     1.0f, 0.0f, 0.33f;
    ADEBUG << "state trans matrix in RadarFrameSupplement is \n"
           << RadarFrameSupplement::state_vars.trans_matrix << std::endl;
    RadarFrameSupplement::state_vars.initialized_ = true;
  }

  AINFO << "Initialized the modest radar detector";
  return true;
}

bool ModestRadarDetector::Detect(  // Conti
    const ContiRadar &raw_obstacles,
    const std::vector<PolygonDType> &map_polygons,
    const RadarDetectorOptions &options,
    std::vector<std::shared_ptr<Object>> *objects) {
  if (objects == nullptr) {
    AERROR << "Objects is nullptr";
    return false;
  }
  ADEBUG << "Modest radar detector.";
  Eigen::Matrix4d radar_pose;
  if (options.radar2world_pose == nullptr) {
    AERROR << "radar2world_pose is nullptr.";
    return false;
  } else {
    radar_pose = *(options.radar2world_pose);
  }
  Eigen::Vector2d main_velocity;
  if (FLAGS_use_navigation_mode) {
    main_velocity[0] = 0;
    main_velocity[1] = 0;
  } else {
    main_velocity[0] = options.car_linear_speed[0];
    main_velocity[1] = options.car_linear_speed[1];
  }

  // object builder
  SensorObjects radar_objects;
  object_builder_.Build(raw_obstacles, radar_pose, main_velocity,
                        &radar_objects);
  radar_objects.timestamp =
      static_cast<double>(raw_obstacles.header().timestamp_sec());
  radar_objects.sensor_type = SensorType::RADAR;

  // roi filter; skipped in navigation mode
  auto &filter_objects = radar_objects.objects;
  RoiFilter(map_polygons, &filter_objects);

  // treatment
  radar_tracker_->Process(radar_objects);
  ADEBUG << "After process, object size: " << radar_objects.objects.size();
  CollectRadarResult(objects);
  ADEBUG << "radar object size: " << objects->size();

  // initialize RadarFrameSupplement for fusion later
  if (!RadarFrameSupplement::state_vars.initialized_) {
    RadarFrameSupplement::state_vars.process_noise(0, 0) *= 10;
    RadarFrameSupplement::state_vars.process_noise(1, 1) *= 10;
    RadarFrameSupplement::state_vars.process_noise(2, 2) *= 10;
    RadarFrameSupplement::state_vars.process_noise(3, 3) *= 10;

    RadarFrameSupplement::state_vars.trans_matrix.block(0, 0, 1, 4) << 1.0f,
        0.0f, 0.33f, 0.0f;
    RadarFrameSupplement::state_vars.trans_matrix.block(1, 0, 1, 4) << 0.0f,
        1.0f, 0.0f, 0.33f;
    ADEBUG << "state trans matrix in RadarFrameSupplement is \n"
           << RadarFrameSupplement::state_vars.trans_matrix << std::endl;
    RadarFrameSupplement::state_vars.initialized_ = true;
  }
  return true;
}

bool ModestRadarDetector::CollectRadarResult(
    std::vector<std::shared_ptr<Object>> *objects) {
  std::vector<RadarTrack> &obs_track = radar_tracker_->GetTracks();
  if (objects == nullptr) {
    AERROR << "objects is nullptr";
    return false;
  }
  for (size_t i = 0; i < obs_track.size(); ++i) {
    std::shared_ptr<Object> object_ptr = std::shared_ptr<Object>(new Object());
    const std::shared_ptr<Object> &object_radar_ptr =
        obs_track[i].GetObsRadar();
    if (config_.use_fp_filter() && object_radar_ptr->is_background) {
      continue;
    }
    object_ptr->clone(*object_radar_ptr);
    object_ptr->tracking_time = obs_track[i].GetTrackingTime();
    object_ptr->track_id = obs_track[i].GetObsId();
    object_ptr->latest_tracked_time = obs_track[i].GetTimestamp();
    objects->push_back(object_ptr);
  }
  return true;
}

void ModestRadarDetector::RoiFilter(
    const std::vector<PolygonDType> &map_polygons,
    std::vector<std::shared_ptr<Object>> *filter_objects) {
  ADEBUG << "Before using hdmap, object size:" << filter_objects->size();
  // use new hdmap
  if (config_.use_had_map()) {
    if (!map_polygons.empty()) {
      int obs_number = 0;
      for (size_t i = 0; i < filter_objects->size(); i++) {
        pcl_util::PointD obs_position;
        obs_position.x = filter_objects->at(i)->center(0);
        obs_position.y = filter_objects->at(i)->center(1);
        obs_position.z = filter_objects->at(i)->center(2);
        if (RadarUtil::IsXyPointInHdmap<pcl_util::PointD>(obs_position,
                                                          map_polygons)) {
          filter_objects->at(obs_number) = filter_objects->at(i);
          obs_number++;
        }
      }
      filter_objects->resize(obs_number);
      ADEBUG << "query hdmap sucessfully!";
    } else {
      ADEBUG << "query hdmap unsuccessfully!";
    }
  }
  ADEBUG << "After using hdmap, object size:" << filter_objects->size();
}

bool ModestRadarDetector::GetSensorTrans(const double query_time, Eigen::Matrix4d* trans) {
  if (!trans) {
    AERROR << "failed to get trans, the trans ptr can not be nullptr";
    return false;
  }

  ros::Time query_stamp(query_time);
  // const double kTf2BuffSize = FLAGS_tf2_buff_in_ms / 1000.0;
  std::string err_msg;
  if (!tf2_listener_.canTransform(FLAGS_radar_tf2_frame_id,
                                FLAGS_radar_tf2_child_frame_id,
                                query_stamp,
                              //  ros::Time(0),
                                &err_msg)) {
    AERROR << "Cannot transform frame: " << FLAGS_radar_tf2_frame_id
            << " to frame " << FLAGS_radar_tf2_child_frame_id
            << " , err: " << err_msg
            << ". Frames: " << tf2_listener_.allFramesAsString();
    return false;
  }

  tf::StampedTransform stamped_transform;
  // geometry_msgs::TransformStamped transform_stamped;
  try {
    tf2_listener_.lookupTransform(FLAGS_radar_tf2_frame_id, 
      FLAGS_radar_tf2_child_frame_id, query_stamp, stamped_transform);
  } catch (tf2::TransformException& ex) {
    AERROR << "Exception: " << ex.what();
    return false;
  }
  Eigen::Affine3d affine_3d;
  tf::transformTFToEigen(stamped_transform, affine_3d);
  *trans = affine_3d.matrix();

  // AWARN_IF(log_radar_) << "get " << FLAGS_radar_tf2_frame_id << " to "
  //         << FLAGS_radar_tf2_child_frame_id << " trans: \n" << *trans;
  return true;
}

}  // namespace apollo
