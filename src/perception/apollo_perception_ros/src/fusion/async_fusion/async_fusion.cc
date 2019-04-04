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

#include "fusion/async_fusion/async_fusion.h"

#include <iomanip>

#include "util/log.h"
#include "util/file.h"
#include "util/time_util.h"

#include "common/perception_gflags.h"

#include "fusion/probabilistic_fusion/pbf_base_track_object_matcher.h"
#include "fusion/probabilistic_fusion/pbf_hm_track_object_matcher.h"
#include "fusion/probabilistic_fusion/pbf_sensor_manager.h"

namespace apollo_perception_standalone {

using apollo_perception_standalone::util::GetProtoFromFile;

AsyncFusion::~AsyncFusion() {
  delete track_manager_;
}

bool AsyncFusion::Init() {
  AERROR << "AsyncFusion should be initialized from passed argument!";
  return false;
  // //track_manager_ = PbfTrackManager::instance();
  // track_manager_ = new PbfTrackManager();
  // CHECK_NOTNULL(track_manager_);

  // if (!GetProtoFromFile(FLAGS_async_fusion_config, &config_)) {
  //   AERROR << "Cannot get config proto from file: " << FLAGS_async_fusion_config;
  //   return false;
  // }

  // /* matching parameters */
  // // TODO(All): match_method is set to hm_matcher, so that line 56 - 65 is
  // // redundant. We should either make match_method configurable or remove those
  // // redundant code.
  // std::string match_method = config_.match_method();
  // if (match_method != "hm_matcher") {
  //   AERROR << "undefined match_method " << match_method
  //          << " and use default hm_matcher";
  // }
  // matcher_.reset(new PbfHmTrackObjectMatcher());
  // if (!matcher_->Init()) {
  //   AERROR << "Failed to initialize " << matcher_->name();
  //   return false;
  // }

  // float max_match_distance = config_.max_match_distance();
  // PbfBaseTrackObjectMatcher::SetMaxMatchDistance(max_match_distance);

  // // track related parameters
  // PbfTrack::SetMaxLidarInvisiblePeriod(config_.max_lidar_invisible_period());
  // PbfTrack::SetMaxRadarInvisiblePeriod(config_.max_radar_invisible_period());
  // PbfTrack::SetMaxCameraInvisiblePeriod(config_.max_camera_invisible_period());
  // PbfTrack::SetMaxRadarConfidentAngle(
  //   config_.max_radar_confident_angle());
  // PbfTrack::SetMinRadarConfidentDistance(
  //   config_.min_radar_confident_distance());
  
  // std::string motion_method = config_.motion_method();
  // if (motion_method == "kf") {
  //   PbfTrack::SetMotionFusionMethod("PbfKalmanMotionFusion");
  // } else if (motion_method == "imf") {
  //   PbfTrack::SetMotionFusionMethod("PbfIMFFusion");
  // } else {
  //   AWARN << "undefined motion_method " << motion_method
  //          << ", use default imf";
  //   PbfTrack::SetMotionFusionMethod("PbfIMFFusion");
  // }

  // // initialize BBAManager
  // if (!DSTInitiator::instance().initialize_bba_manager()) {
  //   AERROR << "failed to initialize BBAManager!";
  //   return false;
  // }

  // return true;
}

bool AsyncFusion::Init(bool log_fusion) {
  AERROR << "AsyncFusion should be initialized from passed argument!";
  return false;
  // log_fusion_ = log_fusion;
  // // AWARN_IF(log_fusion_) << "Fusion: async fusion initialization started";

  // //track_manager_ = PbfTrackManager::instance();
  // track_manager_ = new PbfTrackManager();
  // CHECK_NOTNULL(track_manager_);

  // if (!GetProtoFromFile(FLAGS_async_fusion_config, &config_)) {
  //   AERROR << "Cannot get config proto from file: " << FLAGS_async_fusion_config;
  //   return false;
  // }

  // /* matching parameters */
  // // TODO(All): match_method is set to hm_matcher, so that line 56 - 65 is
  // // redundant. We should either make match_method configurable or remove those
  // // redundant code.
  // std::string match_method = config_.match_method();
  // if (match_method != "hm_matcher") {
  //   AERROR << "undefined match_method " << match_method
  //          << " and use default hm_matcher";
  // }
  // matcher_.reset(new PbfHmTrackObjectMatcher());
  // if (!matcher_->Init()) {
  //   AERROR << "Failed to initialize " << matcher_->name();
  //   return false;
  // }

  // float max_match_distance = config_.max_match_distance();
  // PbfBaseTrackObjectMatcher::SetMaxMatchDistance(max_match_distance);

  // // track related parameters
  // PbfTrack::SetMaxLidarInvisiblePeriod(config_.max_lidar_invisible_period());
  // PbfTrack::SetMaxRadarInvisiblePeriod(config_.max_radar_invisible_period());
  // PbfTrack::SetMaxCameraInvisiblePeriod(config_.max_camera_invisible_period());
  // PbfTrack::SetMaxRadarConfidentAngle(
  //   config_.max_radar_confident_angle());
  // PbfTrack::SetMinRadarConfidentDistance(
  //   config_.min_radar_confident_distance());
  
  // std::string motion_method = config_.motion_method();
  // if (motion_method == "kf") {
  //   PbfTrack::SetMotionFusionMethod("PbfKalmanMotionFusion");
  // } else if (motion_method == "imf") {
  //   PbfTrack::SetMotionFusionMethod("PbfIMFFusion");
  // } else {
  //   AWARN << "Fusion: undefined motion_method " << motion_method
  //          << ", use default imf";
  //   PbfTrack::SetMotionFusionMethod("PbfIMFFusion");
  // }

  // // if (!LidarFrameSupplement::state_vars.initialized_) {
  // //     AERROR << "process noise and trans matrix not initialized for lidar";
  // //     return false;
  // // }
  // LidarFrameSupplement::state_vars.process_noise *= config_.lidar_process_noise_scale_factor();
  // // if (!RadarFrameSupplement::state_vars.initialized_) {
  // //   AERROR << "process noise and trans matrix not initialized for radar";
  // //   return false;
  // // }
  // RadarFrameSupplement::state_vars.process_noise *= config_.radar_process_noise_scale_factor();
  // // if (!CameraFrameSupplement::state_vars.initialized_) {
  // //     AERROR << "process noise and trans matrix not initialized for camera";
  // //     return false;
  // //   }
  // CameraFrameSupplement::state_vars.process_noise *= config_.camera_process_noise_scale_factor();

  // // initialize BBAManager
  // if (!DSTInitiator::instance().initialize_bba_manager()) {
  //   AERROR << "failed to initialize BBAManager!";
  //   return false;
  // }
  
  // AWARN_IF(log_fusion_) << "Fusion: " << name() 
  //   << ", with motion_method: "  << "\033[1;33m" << motion_method << "\033[0m";
  // AWARN_IF(log_fusion_) << "Lidar process noise scale factor: " 
  //   << "\033[1;33m" << config_.lidar_process_noise_scale_factor() << "\033[0m";
  // AWARN_IF(log_fusion_) << "Radar process noise scale factor: " 
  //   << "\033[1;33m" << config_.radar_process_noise_scale_factor() << "\033[0m";
  // AWARN_IF(log_fusion_) << "Camera process noise scale factor: " 
  //   << "\033[1;33m" << config_.camera_process_noise_scale_factor() << "\033[0m";
  // AWARN_IF(log_fusion_) << "Lidar max invisible period: " 
  //   << "\033[1;33m" << config_.max_lidar_invisible_period() << "\033[0m";
  // AWARN_IF(log_fusion_) << "Radar max invisible period: " 
  //   << "\033[1;33m" << config_.max_radar_invisible_period() << "\033[0m";
  // AWARN_IF(log_fusion_) << "Camera max invisible period: " 
  //   << "\033[1;33m" << config_.max_camera_invisible_period() << "\033[0m";
  // return true;
}

bool AsyncFusion::Init(bool log_fusion, std::string &file_path_config) {
  
  log_fusion_ = log_fusion;
  // AWARN_IF(log_fusion_) << "Fusion: async fusion initialization started";

  //track_manager_ = PbfTrackManager::instance();
  track_manager_ = new PbfTrackManager();
  CHECK_NOTNULL(track_manager_);

  std::string file_path_config_async_fusion = 
    file_path_config + "/config/async_fusion_config.pb.txt";

  if (!GetProtoFromFile(file_path_config_async_fusion, &config_)) {
    AERROR << "Cannot get config proto from file: " << file_path_config_async_fusion;
    return false;
  }

  /* matching parameters */
  // TODO(All): match_method is set to hm_matcher, so that line 56 - 65 is
  // redundant. We should either make match_method configurable or remove those
  // redundant code.
  std::string match_method = config_.match_method();
  if (match_method != "hm_matcher") {
    AERROR << "undefined match_method " << match_method
           << " and use default hm_matcher";
  }
  matcher_.reset(new PbfHmTrackObjectMatcher());
  if (!matcher_->Init()) {
    AERROR << "Failed to initialize " << matcher_->name();
    return false;
  }

  float max_match_distance = config_.max_match_distance();
  PbfBaseTrackObjectMatcher::SetMaxMatchDistance(max_match_distance);

  // track related parameters
  PbfTrack::SetMaxLidarInvisiblePeriod(config_.max_lidar_invisible_period());
  PbfTrack::SetMaxRadarInvisiblePeriod(config_.max_radar_invisible_period());
  PbfTrack::SetMaxCameraInvisiblePeriod(config_.max_camera_invisible_period());
  PbfTrack::SetMaxRadarConfidentAngle(
    config_.max_radar_confident_angle());
  PbfTrack::SetMinRadarConfidentDistance(
    config_.min_radar_confident_distance());
  
  std::string motion_method = config_.motion_method();
  if (motion_method == "kf") {
    PbfTrack::SetMotionFusionMethod("PbfKalmanMotionFusion");
  } else if (motion_method == "imf") {
    PbfTrack::SetMotionFusionMethod("PbfIMFFusion");
  } else {
    AWARN << "Fusion: undefined motion_method " << motion_method
           << ", use default imf";
    PbfTrack::SetMotionFusionMethod("PbfIMFFusion");
  }

  // if (!LidarFrameSupplement::state_vars.initialized_) {
  //     AERROR << "process noise and trans matrix not initialized for lidar";
  //     return false;
  // }
  LidarFrameSupplement::state_vars.process_noise *= config_.lidar_process_noise_scale_factor();
  // if (!RadarFrameSupplement::state_vars.initialized_) {
  //   AERROR << "process noise and trans matrix not initialized for radar";
  //   return false;
  // }
  RadarFrameSupplement::state_vars.process_noise *= config_.radar_process_noise_scale_factor();
  // if (!CameraFrameSupplement::state_vars.initialized_) {
  //     AERROR << "process noise and trans matrix not initialized for camera";
  //     return false;
  //   }
  CameraFrameSupplement::state_vars.process_noise *= config_.camera_process_noise_scale_factor();

  // initialize BBAManager
  if (!DSTInitiator::instance().initialize_bba_manager()) {
    AERROR << "failed to initialize BBAManager!";
    return false;
  }
  
  AWARN_IF(log_fusion_) << "Fusion: " << name() 
    << ", with motion_method: "  << "\033[1;33m" << motion_method << "\033[0m";
  AWARN_IF(log_fusion_) << "Lidar process noise scale factor: " 
    << "\033[1;33m" << config_.lidar_process_noise_scale_factor() << "\033[0m";
  AWARN_IF(log_fusion_) << "Radar process noise scale factor: " 
    << "\033[1;33m" << config_.radar_process_noise_scale_factor() << "\033[0m";
  AWARN_IF(log_fusion_) << "Camera process noise scale factor: " 
    << "\033[1;33m" << config_.camera_process_noise_scale_factor() << "\033[0m";
  AWARN_IF(log_fusion_) << "Lidar max invisible period: " 
    << "\033[1;33m" << config_.max_lidar_invisible_period() << "\033[0m";
  AWARN_IF(log_fusion_) << "Radar max invisible period: " 
    << "\033[1;33m" << config_.max_radar_invisible_period() << "\033[0m";
  AWARN_IF(log_fusion_) << "Camera max invisible period: " 
    << "\033[1;33m" << config_.max_camera_invisible_period() << "\033[0m";
  return true;
}

PbfSensorFramePtr AsyncFusion::ConstructFrame(const SensorObjects &frame) {
  PbfSensorFramePtr pbf_frame(new PbfSensorFrame());
  pbf_frame->timestamp = frame.timestamp;
  pbf_frame->sensor2world_pose = frame.sensor2world_pose;
  pbf_frame->sensor_type = frame.sensor_type;
  pbf_frame->sensor_id = GetSensorType(frame.sensor_type);
  pbf_frame->seq_num = frame.seq_num;

  pbf_frame->objects.resize(frame.objects.size());
  for (size_t i = 0; i < frame.objects.size(); i++) {
    std::shared_ptr<PbfSensorObject> obj(new PbfSensorObject());
    obj->timestamp = frame.timestamp;
    obj->sensor_type = frame.sensor_type;
    obj->object->clone(*(frame.objects[i]));
    obj->sensor_id = GetSensorType(frame.sensor_type);
    pbf_frame->objects[i] = obj;
  }
  return pbf_frame;
}

bool AsyncFusion::Fuse(const std::vector<SensorObjects> &multi_sensor_objects,
                       std::vector<std::shared_ptr<Object>> *fused_objects,
                       FusionOptions *options) {
  ACHECK(fused_objects != nullptr) << "parameter fused_objects is nullptr";

  // AWARN_IF(log_fusion_) << "number of sensor objects in async fusion is "
  //       << multi_sensor_objects.size();

  // process all the frames from one of the sensors
  for (const auto &obj : multi_sensor_objects) {
    double fusion_time = obj.timestamp;
    // AWARN_IF(log_fusion_) << "get sensor data " << GetSensorType(obj.sensor_type)
    //       << ", obj_cnt : " << obj.objects.size() << ", " << std::fixed
    //       << std::setprecision(12) << obj.timestamp;

    PbfSensorFramePtr frame = ConstructFrame(obj);

    {
      fusion_mutex_.lock();
      FuseFrame(frame);
      // 4.collect results
      CollectFusedObjects(fusion_time, fused_objects);
      fusion_mutex_.unlock();
    }
  }
  return true;
}

std::string AsyncFusion::name() const { return "AsyncFusion"; }

void AsyncFusion::FuseFrame(PbfSensorFramePtr frame) {
  AWARN_IF(log_fusion_) << "fusing frame: " << frame->sensor_id << ","
        << "object_number: " << frame->objects.size() << ","
        << "timestamp: " << std::fixed << std::setprecision(12)
        << frame->timestamp;
  std::vector<std::shared_ptr<PbfSensorObject>> &objects = frame->objects;
  std::vector<std::shared_ptr<PbfSensorObject>> background_objects;
  std::vector<std::shared_ptr<PbfSensorObject>> foreground_objects;
  DecomposeFrameObjects(objects, &foreground_objects, &background_objects);
  AWARN_IF(log_fusion_) << "There are " << foreground_objects.size() << " foreground objects, "
        << background_objects.size() << " background objects";

  Eigen::Vector3d ref_point = frame->sensor2world_pose.topRightCorner(3, 1);
  FuseForegroundObjects(ref_point, frame->sensor_type, frame->sensor_id,
                        frame->timestamp, &foreground_objects);
  track_manager_->RemoveLostTracks(log_fusion_);
}

void AsyncFusion::CreateNewTracks(
    const std::vector<std::shared_ptr<PbfSensorObject>> &sensor_objects,
    const std::vector<int> &unassigned_ids, 
    const SensorType &sensor_type) {
  for (size_t i = 0; i < unassigned_ids.size(); ++i) {
    int id = unassigned_ids[i];
    // if (is_lidar(sensor_type) || is_camera(sensor_type)) {
    if (is_lidar(sensor_type)) {
      PbfTrackPtr track(new PbfTrack(sensor_objects[id], log_fusion_));
      AWARN_IF(log_fusion_) << "NEW TRACK created, id: " << track->GetTrackId()  
        << " from sensor: " << GetSensorType(sensor_objects[id]->sensor_type);
      track_manager_->AddTrack(track);
    }
    if (is_radar(sensor_type)) {
      float range = sensor_objects[id]->object->radar_supplement->range;
      float angle = sensor_objects[id]->object->radar_supplement->angle;
      if (range < PbfTrack::GetMinRadarConfidentDistance()
          || angle > PbfTrack::GetMaxRadarConfidentAngle()) {
        continue;
      }
      PbfTrackPtr track(new PbfTrack(sensor_objects[id], log_fusion_));
      AWARN_IF(log_fusion_) << "\033[1;31mNEW TRACK\033[0m created, id: " << track->GetTrackId()  
        << " from sensor: " << GetSensorType(sensor_objects[id]->sensor_type) 
        << " at range: " << range << " and angle: " << angle;
      track_manager_->AddTrack(track);
    }
  }
}

void AsyncFusion::UpdateAssignedTracks(
    const std::vector<std::shared_ptr<PbfSensorObject>> &sensor_objects,
    const std::vector<std::pair<int, int>> &assignments,
    const std::vector<double> &track_object_dist,
    std::vector<PbfTrackPtr> const *tracks) {
  for (size_t i = 0; i < assignments.size(); ++i) {
    int local_track_index = assignments[i].first;
    int local_obj_index = assignments[i].second;
    AINFO << "local track index " << local_track_index << " local object index "
          << local_obj_index;
    if (is_lidar(sensor_objects[local_obj_index]->sensor_type)) {
      tracks->at(local_track_index)->SetFusedLidarStatus(true);  // fused_lidar as true
    } else if (is_radar(sensor_objects[local_obj_index]->sensor_type)) {
      tracks->at(local_track_index)->SetFusedRadarStatus(true);  // fused_radar as true
    } else if (is_camera(sensor_objects[local_obj_index]->sensor_type)) {
      tracks->at(local_track_index)->SetFusedCamStatus(true);  // fused_cam as true
    }
    tracks->at(local_track_index)
        ->UpdateWithSensorObject(sensor_objects[local_obj_index],
                                 track_object_dist[local_track_index]);
  }
}

void AsyncFusion::UpdateUnassignedTracks(
    const std::vector<int> &unassigned_tracks,
    const std::vector<double> &track_object_dist, const SensorType &sensor_type,
    const std::string &sensor_id, const double timestamp,
    std::vector<PbfTrackPtr> *tracks) {
  for (size_t i = 0; i < unassigned_tracks.size(); ++i) {
    int local_track_index = unassigned_tracks[i];
    if (is_lidar(sensor_type)) {
      tracks->at(local_track_index)->SetFusedLidarStatus(false);  // fused_lidar as false
    } else if (is_radar(sensor_type)) {
      tracks->at(local_track_index)->SetFusedRadarStatus(false);  // fused_radar as false
    } else if (is_camera(sensor_type)) {
      tracks->at(local_track_index)->SetFusedCamStatus(false);  // fused_cam as false
    }
    tracks->at(local_track_index)
        ->UpdateWithoutSensorObject(sensor_type, sensor_id,
                                    track_object_dist[local_track_index],
                                    timestamp);
  }
}

void AsyncFusion::CollectFusedObjects(
    double timestamp, std::vector<std::shared_ptr<Object>> *fused_objects) {
  if (fused_objects == nullptr) {
    return;
  }
  fused_objects->clear();

  int fg_obj_num = 0;
  std::vector<PbfTrackPtr> &tracks = track_manager_->GetTracks();
  for (size_t i = 0; i < tracks.size(); i++) {
    std::shared_ptr<PbfSensorObject> fused_object = tracks[i]->GetFusedObject();
    std::shared_ptr<Object> obj(new Object());
    obj->clone(*(fused_object->object));
    obj->track_id = tracks[i]->GetTrackId();
    std::shared_ptr<PbfSensorObject> pobj = tracks[i]->GetLidarObject("lidar");
    if (pobj != nullptr) {
      obj->local_lidar_track_id = pobj->object->track_id;
      obj->local_lidar_track_ts = pobj->timestamp;
    }
    pobj = tracks[i]->GetCameraObject("camera");
    if (pobj != nullptr) {
      obj->local_camera_track_id = pobj->object->track_id;
      obj->local_camera_track_ts = pobj->timestamp;
    }
    pobj = tracks[i]->GetRadarObject("radar");
    if (pobj != nullptr) {
      obj->local_radar_track_id = pobj->object->track_id;
      obj->local_radar_track_ts = pobj->timestamp;
    }
    obj->latest_tracked_time = timestamp;
    obj->tracking_time = tracks[i]->GetTrackingPeriod();
    fused_objects->push_back(obj);
    fg_obj_num++;
  }

  // AWARN_IF(log_fusion_) << "collect objects : fg_track_cnt = " << tracks.size()
  //       << ", timestamp = " << GLOG_TIMESTAMP(timestamp);
}

void AsyncFusion::DecomposeFrameObjects(
    const std::vector<std::shared_ptr<PbfSensorObject>> &frame_objects,
    std::vector<std::shared_ptr<PbfSensorObject>> *foreground_objects,
    std::vector<std::shared_ptr<PbfSensorObject>> *background_objects) {
  foreground_objects->clear();
  background_objects->clear();
  for (size_t i = 0; i < frame_objects.size(); i++) {
    if (frame_objects[i]->object->is_background) {
      background_objects->push_back(frame_objects[i]);
    } else {
      foreground_objects->push_back(frame_objects[i]);
    }
  }
}

void AsyncFusion::FuseForegroundObjects(
    const Eigen::Vector3d &ref_point, const SensorType &sensor_type,
    const std::string &sensor_id, const double timestamp,
    std::vector<std::shared_ptr<PbfSensorObject>> *foreground_objects) {
  std::vector<int> unassigned_tracks;
  std::vector<int> unassigned_objects;
  std::vector<std::pair<int, int>> assignments;

  std::vector<PbfTrackPtr> &tracks = track_manager_->GetTracks();

  TrackObjectMatcherOptions options;
  options.ref_point = &ref_point;

  std::vector<double> track2measurements_dist;
  std::vector<double> measurement2tracks_dist;
  matcher_->Match(tracks, *foreground_objects, options, &assignments,
                  &unassigned_tracks, &unassigned_objects,
                  &track2measurements_dist, &measurement2tracks_dist);

  AWARN_IF(log_fusion_) << "---------- Fusion matching:"
        << " fg_track_cnt = " << tracks.size()
        << ", fg_obj_cnt = " << foreground_objects->size()
        << ", assignement = " << assignments.size()
        << ", unassigned_track_cnt = " << unassigned_tracks.size()
        << ", unassigned_obj_cnt = " << unassigned_objects.size();

  UpdateAssignedTracks(*foreground_objects, assignments,
                       track2measurements_dist, &tracks);

  UpdateUnassignedTracks(unassigned_tracks, track2measurements_dist,
                         sensor_type, sensor_id, timestamp, &tracks);

  // fixme:zhangweide only create new track if it is camera sensor
  CreateNewTracks(*foreground_objects, unassigned_objects, sensor_type);
}

}  // namespace apollo
