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

#include "fusion/probabilistic_fusion/pbf_imf_fusion.h"  // NOLINT

#include "util/log.h"
#include "common/types.h"

namespace apollo_perception_standalone {

PbfIMFFusion::PbfIMFFusion() {
  initialized_ = false;
  name_ = "PbfInformationMotionFusion";
}

PbfIMFFusion::~PbfIMFFusion() {}

void PbfIMFFusion::Initialize(const Eigen::Vector3d& anchor_point,
                              const Eigen::Vector3d& velocity) {
  belief_anchor_point_ = anchor_point;
  belief_velocity_ = velocity;
}

void PbfIMFFusion::Initialize(
    const std::shared_ptr<PbfSensorObject> new_object) {
  if (new_object == nullptr) {
    AERROR << "Initialize PbfInformationMotionFusion with null sensor object";
    return;
  }
  if (new_object->object == nullptr) {
    AERROR << "Initialize PbfInformationMotionFusion with null object";
    return;
  }

  belief_anchor_point_ = new_object->object->anchor_point;
  belief_velocity_ = new_object->object->velocity;

  // initialize states to the states of the detected obstacle
  posteriori_state_(0) = belief_anchor_point_(0);
  posteriori_state_(1) = belief_anchor_point_(1);
  posteriori_state_(2) = belief_velocity_(0);
  posteriori_state_(3) = belief_velocity_(1);
  priori_state_ = posteriori_state_;
  omega_matrix_ = new_object->object->state_uncertainty;
  omega_matrix_ = omega_matrix_.inverse().eval();
  xi_ = omega_matrix_ * posteriori_state_;
  q_matrix_.setIdentity();
  // ADEBUG << "PBFIMF pbf imf filter initial state is " << posteriori_state_(0)
  //        << " " << posteriori_state_(1) << " " << posteriori_state_(2) << " "
  //        << posteriori_state_(3)
  //        << " for object info: " << new_object->object->ToString();
  // std::cerr << "next track\n" << std::endl;
  // std::cerr << "PBFIMF pbf imf initial uncertainty set "
  //           << new_object->object->state_uncertainty << std::endl;
  // std::cerr << "PBFIMF pbf imf omega matrix is " << omega_matrix_ << std::endl;
  cov_matrix_ = omega_matrix_.inverse();
  CacheSensorObjects(new_object);
  initialized_ = true;
}

void PbfIMFFusion::Predict(Eigen::Vector3d* anchor_point,
                           Eigen::Vector3d* velocity, const double time_diff) {
  *anchor_point = belief_anchor_point_ + belief_velocity_ * time_diff;
  *velocity = belief_velocity_;
}

void PbfIMFFusion::UpdateWithObject(
    const std::shared_ptr<PbfSensorObject> new_object, const double time_diff) {
  if (new_object == nullptr) {
    AERROR << "update PbfInformationMotionFusion with null sensor object";
    return;
  }
  if (new_object->object == nullptr) {
    AERROR << "update PbfInformationMotionFusion with null object";
    return;
  }

  // remove outdated object stored in cached_sensor_objects_
  RemoveOutdatedSensorObjects(new_object->timestamp);

  // print some debug information for debugging
  AWARN_IF(log_fusion_imf_) << "\033[1;31mPBFIMF: STARTED\033[0m\n";
  AWARN_IF(log_fusion_imf_) << "PBFIMF: previous state: \n" 
         << belief_anchor_point_(0) << " " << belief_anchor_point_(1) << " " << belief_anchor_point_(2)
          << " " << belief_velocity_(0) << " " << belief_velocity_(1) << " " << belief_velocity_(2);
  AWARN_IF(log_fusion_imf_) << "PBFIMF: previous covariance: \n" 
         << cov_matrix_;
  AWARN_IF(log_fusion_imf_) << "PBFIMF: previous timestamp: " << std::fixed
         << std::setprecision(15) << last_fuse_timestamp;
  AWARN_IF(log_fusion_imf_) << "PBFIMF: new object information: \n"
         << new_object->object->ToString();
  AWARN_IF(log_fusion_imf_) << "PBFIMF: new object timestamp: " << std::fixed
         << std::setprecision(15) << new_object->timestamp;
  AWARN_IF(log_fusion_imf_) << "PBFIMF: time diff is " << time_diff;

  // ************************************************************* //
  // *************** Priori State ******************************** //
  // ************************************************************* //
  // compute priori
  a_matrix_.setIdentity();  // state transition matrix with time
  a_matrix_(0, 2) = time_diff;
  a_matrix_(1, 3) = time_diff;
  q_matrix_.setIdentity();  // process noise 
  q_matrix_(0, 0) *= 0.1;
  q_matrix_(1, 1) *= 0.1;
  q_matrix_(2, 2) *= 0.1;
  q_matrix_(3, 3) *= 0.1;
  // priori state
  priori_state_ = a_matrix_ * posteriori_state_;
  Eigen::Matrix4d old_omega_ = omega_matrix_;
  omega_matrix_ = (a_matrix_ * cov_matrix_ * a_matrix_.transpose() + q_matrix_);
  // priori covariance matrix
  Eigen::Matrix4d pred_omega_ = omega_matrix_;
  // aliasing in eigen: http://eigen.tuxfamily.org/dox/group__TopicAliasing.html
  // priori information matrix
  omega_matrix_ = omega_matrix_.inverse().eval();
  // priori information vector
  xi_ = omega_matrix_ * priori_state_;
  // debug output
  AWARN_IF(log_fusion_imf_) << "\033[1;31mPBFIMF: PRIORI\033[0m\n";
  AWARN_IF(log_fusion_imf_) << "PBFIMF: priori state: \n" << priori_state_(0) << " "
         << priori_state_(1) << " " << priori_state_(2) << " "
         << priori_state_(3);
  AWARN_IF(log_fusion_imf_) << "PBFIMF: priori covariance: \n" << pred_omega_;
  AWARN_IF(log_fusion_imf_) << "PBFIMF: priori inf vec: \n" << xi_;
  AWARN_IF(log_fusion_imf_) << "PBFIMF: priori inf mat: \n" <<  omega_matrix_;
  // std::cerr << "PBFIMF: old omega is \n" << old_omega_ << std::endl;
  // std::cerr << "PBFIMF:predicted omega is \n" << pred_omega_ << std::endl;
  // std::cerr << "PBFIMF: old omega inv is \n"
  //           << old_omega_.inverse() << std::endl;
  // std::cerr << "PBFIMF:predicted omega inv is \n"
  //           << pred_omega_.inverse() << std::endl;
  
  // ************************************************************* //
  // *************** Sensor Matrix ******************************* //
  // ************************************************************* //
  // sensor level processor noise matrix and trans matrix
  const Eigen::Matrix4d* sensor_processor_noise;
  const Eigen::Matrix4d* sensor_transition_matrix;
  if (is_camera(new_object->sensor_type)) {  // camera sensor
    // std::cerr << "senser type : CAMERA\n" << std::endl;
    AINFO << "senser type : CAMERA";
    belief_anchor_point_ = new_object->object->center;
    belief_velocity_ = new_object->object->velocity;
    if (!CameraFrameSupplement::state_vars.initialized_) {
      AERROR << "process noise and trans matrix not initialized for camera";
      return;
    }
    sensor_processor_noise = &(CameraFrameSupplement::state_vars.process_noise);
    sensor_transition_matrix =
        &(CameraFrameSupplement::state_vars.trans_matrix);
  } else if (is_radar(new_object->sensor_type)) {  // radar sensor
    // std::cerr << "senser type : RADAR\n" << std::endl;
    AINFO << "senser type : RADAR";
    // belief_anchor_point_ = new_object->object->center;
    // belief_velocity_ = new_object->object->velocity;

    // for radar, we don't set externally yet, just use default value
    if (!RadarFrameSupplement::state_vars.initialized_) {
      AERROR << "process noise and trans matrix not initialized for radar";
      return;
    }
    sensor_processor_noise = &(RadarFrameSupplement::state_vars.process_noise);
    sensor_transition_matrix = &(RadarFrameSupplement::state_vars.trans_matrix);
  } else if (is_lidar(new_object->sensor_type)) {  // lidar sensor
    belief_anchor_point_ = new_object->object->center;
    belief_velocity_ = new_object->object->velocity;
    if (!LidarFrameSupplement::state_vars.initialized_) {
      AERROR << "process noise and trans matrix not initialized for velodyne64";
      return;
    }
    // if full trust in lidar detection, return right away
    // return;
    sensor_processor_noise = &(LidarFrameSupplement::state_vars.process_noise);
    sensor_transition_matrix = &(LidarFrameSupplement::state_vars.trans_matrix);
  } else {
    AERROR << "Unsupported sensor type, setting using default value";
    return;
  }

  // ************************************************************* //
  // *************** Posteriori State **************************** //
  // ************************************************************* //
  const std::shared_ptr<PbfSensorObject> sensor_object =
      GetSensorLatestCache(new_object->sensor_type);
  if (!is_lidar(new_object->sensor_type) && sensor_object != nullptr) {  // not lidar && last observation exists
    // previous observation
    Eigen::Matrix4d cov_sensor_prev = Eigen::Matrix4d::Identity();
    Eigen::Vector4d state_sensor_prev = Eigen::Vector4d::Zero();
    double timestamp_sensor_prev = sensor_object->timestamp;
    // propagate prev observation to fuse_timestamp
    AWARN_IF(log_fusion_imf_) << "\033[1;31mPBFIMF: PREV OBSERVATION\033[0m\n";
    if (!ObtainSensorPrediction(sensor_object->object, timestamp_sensor_prev,
                                *sensor_processor_noise,
                                *sensor_transition_matrix, &state_sensor_prev,
                                &cov_sensor_prev)) {
      AERROR << "obtain previous sensor prediction fails";
      return;
    }
    // current observation
    Eigen::Matrix4d cov_sensor = Eigen::Matrix4d::Identity();
    Eigen::Vector4d state_sensor = Eigen::Vector4d::Zero();
    double timestamp_sensor = new_object->timestamp;
    // propagate current observation to fuse_timestamp
    AWARN_IF(log_fusion_imf_) << "\033[1;31mPBFIMF: CURR OBSERVATION\033[0m\n";
    if (!ObtainSensorPrediction(
            new_object->object, timestamp_sensor, *sensor_processor_noise,
            *sensor_transition_matrix, &state_sensor, &cov_sensor)) {
      AERROR << "obtain current sensor prediction fails";
      return;
    }

    Eigen::Matrix4d cov_sensor_inverse = cov_sensor.inverse();
    Eigen::Matrix4d cov_sensor_prev_inverse = cov_sensor_prev.inverse();
    // AWARN_IF(log_fusion_imf_) << "PBFIMF: state sensor " 
    //         << state_sensor(0) << " " << state_sensor(1);
    // AWARN_IF(log_fusion_imf_) << "PBFIMF: state sensor prev " 
    //         << state_sensor_prev(0) << " " << state_sensor_prev(1);
    // std::cerr << "cov sensor prev " << cov_sensor_prev << std::endl;
    // std::cerr << "cov sensor " << cov_sensor << std::endl;
    // AWARN_IF(log_fusion_imf_) << "PBFIMF: cov sensor inverse " << cov_sensor_inverse;
    // AWARN_IF(log_fusion_imf_) << "PBFIMF: cov sensor prev inverse " << cov_sensor_prev_inverse;
    
    AWARN_IF(log_fusion_imf_) << "\033[1;31mPBFIMF: UPDATE\033[0m\n";

    // information matrix update
    omega_matrix_ =
        omega_matrix_ + (cov_sensor_inverse - cov_sensor_prev_inverse);
    AWARN_IF(log_fusion_imf_) << "PBFIMF: inf mat delta: \n"
            << (cov_sensor_inverse - cov_sensor_prev_inverse);
    
    // information vector update
    xi_ = xi_ + (cov_sensor_inverse * state_sensor -
                 cov_sensor_prev_inverse * state_sensor_prev);
    AWARN_IF(log_fusion_imf_) << "PBFIMF: inf vec delta: \n "
          << (cov_sensor_inverse * state_sensor -
              cov_sensor_prev_inverse * state_sensor_prev);

  } else {  //  is lidar || sensor_object from GetSensorLatestCache is nullptr
    if (sensor_object == nullptr) {
      // this case is weird, might lead to unexpected situation
      AWARN_IF(log_fusion_imf_) 
      << "\033[1;31mSensor data deprecation in Fusion, GetSensorLatestCache is nullptr\033[0m";
    }
    
    Eigen::Matrix4d cov_sensor = Eigen::Matrix4d::Identity();
    Eigen::Vector4d state_sensor = Eigen::Vector4d::Zero();
    double timestamp_sensor = new_object->timestamp;

    if (!ObtainSensorPrediction(
            new_object->object, timestamp_sensor, *sensor_processor_noise,
            *sensor_transition_matrix, &state_sensor, &cov_sensor)) {
      AERROR << "obtain current sensor prediction fails";
      return;
    }

    Eigen::Matrix4d cov_sensor_inverse = cov_sensor.inverse();
    omega_matrix_ = omega_matrix_ + cov_sensor_inverse;
    AWARN_IF(log_fusion_imf_) << "PBFIMF: inf mat delta: \n"
            << cov_sensor_inverse;
    xi_ = xi_ + cov_sensor_inverse * state_sensor;
    AWARN_IF(log_fusion_imf_) << "PBFIMF: inf vec delta: \n "
            << cov_sensor_inverse * state_sensor;

    // TODO: why 0.5?
    // omega_matrix_ = 0.5 * omega_matrix_ + 0.5 * cov_sensor.inverse();
    // xi_ = 0.5 * xi_ + 0.5 * cov_sensor.inverse() * state_sensor;

    // reset to curr observation
    // cov_matrix_ = new_object->object->state_uncertainty;
    // omega_matrix_ = cov_matrix_.inverse();
    // state_sensor(0) = new_object->object->anchor_point(0);
    // state_sensor(1) = new_object->object->anchor_point(1);
    // state_sensor(2) = new_object->object->velocity(0);
    // state_sensor(3) = new_object->object->velocity(1);
    // xi_ = omega_matrix_ * state_sensor;
  }

  AWARN_IF(log_fusion_imf_) << "PBFIMF: updated inf mat: \n"
          << omega_matrix_;
  AWARN_IF(log_fusion_imf_) << "PBFIMF: updated inf vec: \n" 
          << xi_;

  // update and singularity check for covariance matrix
  AdjustCovMatrix();

  // calculate posteriori state
  posteriori_state_ = cov_matrix_ * xi_;

  // update belief
  belief_anchor_point_(0) = posteriori_state_(0);
  belief_anchor_point_(1) = posteriori_state_(1);
  belief_velocity_(0) = posteriori_state_(2);
  belief_velocity_(1) = posteriori_state_(3);
  AWARN_IF(log_fusion_imf_) << "\033[1;31mPBFIMF: POSTERIORI\033[0m\n";
  AWARN_IF(log_fusion_imf_) << "PBFIMF: new state: \n" 
          << belief_anchor_point_(0) << " " << belief_anchor_point_(1) << " " << belief_anchor_point_(2)
          << " " << belief_velocity_(0) << " " << belief_velocity_(1) << " " << belief_velocity_(2);
  AWARN_IF(log_fusion_imf_) << "PBFIMF: new cov_matrix: \n" << cov_matrix_;
  // if (std::abs(posteriori_state_(2)) > 5 ||
  //     std::abs(posteriori_state_(3)) > 5) {
  //   std::cerr << "PBFIMF:posterior state \n" << posteriori_state_ << std::endl;
  // }

  // save observation
  CacheSensorObjects(new_object);
}

bool PbfIMFFusion::AdjustCovMatrix() {
  cov_matrix_ = omega_matrix_.inverse();
  es_.compute(cov_matrix_);
  Eigen::Vector4d eigenvalues = es_.eigenvalues().transpose();
  // std::cerr << "adjust eigen values for covariance matrix\n";
  // std::cerr << "eigen values before \n" << eigenvalues << std::endl;
  if (eigenvalues.minCoeff() > 0) return false;

  eigenvalues = eigenvalues.cwiseMax(cov_eigen_thresh_);
  // std::cerr << "eigen values after \n" << eigenvalues << std::endl;
  Eigen::Matrix4d diagonal = Eigen::Matrix4d::Identity();
  diagonal(0, 0) = eigenvalues(0);
  diagonal(1, 1) = eigenvalues(1);
  diagonal(2, 2) = eigenvalues(2);
  diagonal(3, 3) = eigenvalues(3);
  Eigen::Matrix4d eigenvectors = es_.eigenvectors();
  cov_matrix_ = eigenvectors * diagonal * eigenvectors.inverse();
  return true;
}

/**
 * obtain sensor level prediction to global fusion arrival time
 * @param obj
 * @param sensor_timestamp
 * @param process_noise
 * @param trans_matrix
 * @param state_pre
 * @param cov_pre
 */
bool PbfIMFFusion::ObtainSensorPrediction(std::shared_ptr<Object> object,
                                          double sensor_timestamp,
                                          const Eigen::Matrix4d& process_noise,
                                          const Eigen::Matrix4d& trans_matrix,
                                          Eigen::Vector4d* state_pre,
                                          Eigen::Matrix4d* cov_pre) {
  Eigen::Matrix<double, 4, 4>& cov = object->state_uncertainty;
  Eigen::Matrix<double, 4, 1> state;

  // state: x,y,vx,vy
  state(0) = object->center(0);
  state(1) = object->center(1);
  state(2) = object->velocity(0);
  state(3) = object->velocity(1);

  double time_diff = fuse_timestamp - sensor_timestamp;

  AWARN_IF(log_fusion_imf_) << "PBFIMF: OBTAIN PREDICT, fuse_timestamp: " 
          << std::fixed << std::setprecision(15) << fuse_timestamp;
  AWARN_IF(log_fusion_imf_) << "PBFIMF: OBTAIN PREDICT, sensor_timestamp: " 
          << std::fixed << std::setprecision(15) << sensor_timestamp;
  AWARN_IF(log_fusion_imf_) << "PBFIMF: OBTAIN PREDICT, time diff: " 
          << std::fixed << std::setprecision(15) << time_diff;
  AWARN_IF(log_fusion_imf_) << "PBFIMF: OBTAIN PREDICT: state is\n " << state;
  AWARN_IF(log_fusion_imf_) << "PBFIMF: OBTAIN PREDICT: cov is\n " << cov;
  AWARN_IF(log_fusion_imf_) << "PBFIMF: OBTAIN PREDICT: process noise is\n " << process_noise;

  // trans_matrix is F matrix for state transition
  // p_pre is sensor level covariance prediction P(ki|ki-1)
  // state_pre is sensor level state prediction x(ki|ki-1)
  Eigen::Matrix4d trans_matrix_time = trans_matrix;
  trans_matrix_time(0, 2) = time_diff;
  trans_matrix_time(1, 3) = time_diff;
  (*state_pre) = trans_matrix_time * state;
  (*cov_pre) =
      trans_matrix_time * cov * trans_matrix_time.transpose() + process_noise;
  
  AWARN_IF(log_fusion_imf_) << "PBFIMF: OBTAIN PREDICT: trans matrix time: \n "
          << trans_matrix_time;
  AWARN_IF(log_fusion_imf_) << "PBFIMF: OBTAIN PREDICT: predicted state pre: \n " << *state_pre;
  AWARN_IF(log_fusion_imf_) << "PBFIMF: OBTAIN PREDICT: predicted cov pre: \n " << *cov_pre;
  
  return true;
}

void PbfIMFFusion::UpdateWithoutObject(const double time_diff) {
  belief_anchor_point_ = belief_anchor_point_ + belief_velocity_ * time_diff;
}

void PbfIMFFusion::GetState(Eigen::Vector3d* anchor_point,
                            Eigen::Vector3d* velocity) {
  *anchor_point = belief_anchor_point_;
  *velocity = belief_velocity_;
}

void PbfIMFFusion::GetUncertainty(Eigen::Matrix3d* position_uncertainty,
                                  Eigen::Matrix3d* velocity_uncertainty) {
  *position_uncertainty << 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01;
  *velocity_uncertainty << 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01;
  (*position_uncertainty).topLeftCorner(2, 2) =
      omega_matrix_.inverse().topLeftCorner(2, 2);
  (*velocity_uncertainty).topLeftCorner(2, 2) =
      omega_matrix_.inverse().block<2, 2>(2, 2);
}

void PbfIMFFusion::CacheSensorObjects(
    const std::shared_ptr<PbfSensorObject> new_object) {
  const SensorType& type = new_object->sensor_type;
  auto it = cached_sensor_objects_.find(type);
  if (it != cached_sensor_objects_.end()) {
    it->second.push(new_object);
  } else {
    std::queue<std::shared_ptr<PbfSensorObject>> objects;
    objects.push(new_object);
    cached_sensor_objects_[type] = objects;
  }
}

void PbfIMFFusion::RemoveOutdatedSensorObjects(const double timestamp) {
  auto it = cached_sensor_objects_.begin();
  for (; it != cached_sensor_objects_.end(); ++it) {
    double time_invisible = 0.0;

    // if (it->first == SensorType::VELODYNE_64) {
    if (is_lidar(it->first)) {
      time_invisible = PbfTrack::GetMaxLidarInvisiblePeriod();
    // } else if (it->first == SensorType::RADAR) {
    } else if (is_radar(it->first)) {  
      time_invisible = PbfTrack::GetMaxRadarInvisiblePeriod();
    // } else if (it->first == SensorType::CAMERA) {
    } else if (is_camera(it->first)) {
      time_invisible = PbfTrack::GetMaxCameraInvisiblePeriod();
    } else {
      AERROR << "getting time_insivible failed, unexpected sensor type!";
    }

    auto& objects = it->second;
    while (!objects.empty()) {
      const auto& object = objects.front();
      if (timestamp - object->timestamp > time_invisible) {
        objects.pop();
        AWARN_IF(log_fusion_imf_)<< "\033[1;31mPBFIMF: CACHED OBSERVATION REMOVED\033[0m\n";
        AWARN_IF(log_fusion_imf_) << "PBFIMF: CACHED OBSERVATION REMOVED, track_id: " 
          << object->object->track_id;
        AWARN_IF(log_fusion_imf_) << "PBFIMF: CACHED OBSERVATION REMOVED, curr obs timestamp: " 
          << std::fixed << std::setprecision(15) << timestamp;
        AWARN_IF(log_fusion_imf_) << "PBFIMF: CACHED OBSERVATION REMOVED, prev obs timestamp: " 
          << std::fixed << std::setprecision(15) << object->timestamp;
        AWARN_IF(log_fusion_imf_) << "PBFIMF: CACHED OBSERVATION REMOVED, time dff: " 
          << std::fixed << std::setprecision(15) << timestamp - object->timestamp;
        AWARN_IF(log_fusion_imf_) << "PBFIMF: CACHED OBSERVATION REMOVED, invisible threshold: " 
          << std::fixed << std::setprecision(15) << time_invisible;
      } else {
        break;
      }
    }
    AWARN_IF(log_fusion_imf_) << "PBFIMF: CACHED OBSERVATION REMOVED, remaining cache size of " 
          << GetSensorType(it->first) << " is: "  << objects.size();
  }
}

std::shared_ptr<PbfSensorObject> PbfIMFFusion::GetSensorLatestCache(
    const SensorType type) {
  auto it = cached_sensor_objects_.find(type);
  if (it != cached_sensor_objects_.end()) {
    const auto& objects = it->second;
    if (!objects.empty()) {
      return objects.back();
    }
  }
  return nullptr;
}

void PbfIMFFusion::SetState(const Eigen::Vector3d& anchor_point,
                            const Eigen::Vector3d& velocity) {
  belief_anchor_point_ = anchor_point;
  belief_velocity_ = velocity;
}

}  // namespace apollo
