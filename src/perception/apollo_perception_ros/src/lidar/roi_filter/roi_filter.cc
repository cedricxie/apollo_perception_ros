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

#include "lidar/roi_filter/roi_filter.h"

#include <algorithm>
#include <limits>

#include "util/file.h"

#include "common/pcl_types.h"

namespace apollo_perception_standalone {

bool ROIFilter::Init(std::string &file_path_roi_filter_config) {
  if (!util::GetProtoFromFile(file_path_roi_filter_config, &config_)) {
    AERROR << "Cannot get config proto from file: "
           << file_path_roi_filter_config;
    return false;
  }
  roi_x_min_ = config_.roi_x_min();
  roi_x_max_ = config_.roi_x_max();
  roi_y_min_ = config_.roi_y_min();
  roi_y_max_ = config_.roi_y_max();
  roi_z_min_ = config_.roi_z_min();
  roi_z_max_ = config_.roi_z_max();
  dist_gplane_ = config_.dist_gplane();
  AINFO << "Successfully get config proto from file: "
           << file_path_roi_filter_config;
  return true;
}

bool ROIFilter::Filter(const ROIFilterOptions &roi_filter_options,
                       const pcl_util::VPointCloudPtr &cloud_ptr, 
                       pcl_util::VPointCloudPtr &filtered_cloud_all_ptr,
                       pcl_util::PointIndicesPtr &filtered_cloud_indices_ptr) {
  FilterROI(roi_filter_options, cloud_ptr);
  *filtered_cloud_all_ptr = *filtered_cloud_all_ptr_;
  *filtered_cloud_indices_ptr = *filtered_cloud_indices_ptr_;
  return true;
}

void ROIFilter::FilterROI(const ROIFilterOptions& roi_filter_options,
                          const pcl_util::VPointCloudPtr &cloud_ptr) {
  filtered_cloud_all_ptr_.reset(new pcl_util::VPointCloud(*cloud_ptr));
  filtered_cloud_indices_ptr_.reset(new pcl::PointIndices());
  // ----------------------------------------------------------------- //
  // ------------- X, Y, and Z PassThrough Filter ---------------------//
  // ----------------------------------------------------------------- //
  // RoI Filter by X and Y Threshold
  if (true) {
    pcl::PassThrough<pcl_util::VPoint> cloudXFilter, cloudYFilter;
    cloudXFilter.setInputCloud(filtered_cloud_all_ptr_);
    cloudXFilter.setFilterFieldName("x");
    cloudXFilter.setFilterLimits(roi_x_min_, roi_x_max_);
    //pass.setFilterLimitsNegative (true); // false: keep within limit; true: keep outside of limit
    cloudXFilter.filter(*filtered_cloud_all_ptr_);
    cloudYFilter.setInputCloud(filtered_cloud_all_ptr_);
    cloudYFilter.setFilterFieldName("y");
    cloudYFilter.setFilterLimits(roi_y_min_, roi_y_max_);
    cloudYFilter.filter(*filtered_cloud_all_ptr_);
  }

  // RoI Filter by Z Threshold
  for (int i = 0; i < filtered_cloud_all_ptr_->points.size(); i++)
  {
    if (filtered_cloud_all_ptr_->points[i].z > roi_z_min_) {
      filtered_cloud_indices_ptr_->indices.push_back(i);
    }
  }
  // pcl_util::VPointCloudPtr cloudPtr_upperZ(new pcl_util::VPointCloud);
  // pcl_util::VPointCloudPtr cloudPtr_lowerZ(new pcl_util::VPointCloud);
  // pcl::PassThrough<pcl_util::VPoint> cloudZFilter;
  // cloudZFilter.setInputCloud(filtered_cloud_all_ptr_);
  // cloudZFilter.setFilterFieldName("z");
  // cloudZFilter.setFilterLimits(roi_z_min_, roi_z_max_);
  // cloudZFilter.filter(*cloudPtr_upperZ); // points between roi_z_min_ and roi_z_max_
  // cloudZFilter.setFilterLimitsNegative (true);
  // cloudZFilter.filter(*cloudPtr_lowerZ); // points below roi_z_min_ and above roi_z_max_
  // AWARN << " roi_z_min_: " << roi_z_min_ << " roi_z_max_: " << roi_z_max_
  //   << " cloudPtr_upperZ size: " << cloudPtr_upperZ->points.size()
  //   << " cloudPtr_lowerZ size: " << cloudPtr_lowerZ->points.size() ;

  // ----------------------------------------------------------------- //
  // ------------- Ground Plane Segmentation --------------------------//
  // ----------------------------------------------------------------- //
  // // 1. Ground plane estimation:
  // Eigen::VectorXf ground_coeffs;
  // ground_coeffs.resize(4);
  // std::vector<int> cloudPtr_lowerZ_indices;
  // for (unsigned int i = 0; i < cloudPtr_lowerZ->points.size(); i++)
  //   cloudPtr_lowerZ_indices.push_back(i);
  // pcl::SampleConsensusModelPlane<pcl_util::VPoint> model_plane(cloudPtr_lowerZ);
  // model_plane.computeModelCoefficients(cloudPtr_lowerZ_indices, ground_coeffs);
  // AWARN << "Ground plane: " << ground_coeffs(0) << " " << ground_coeffs(1) << " " 
  //   << ground_coeffs(2) << " " << ground_coeffs(3) << std::endl;

  // // 2. Plane Segmentation @ http://pointclouds.org/documentation/tutorials/planar_segmentation.php
  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  // pcl::PointIndices::Ptr inliers_seg (new pcl::PointIndices);
  // pcl::SACSegmentation<pcl_util::VPoint> seg; // Create the segmentation object
  // seg.setOptimizeCoefficients (true); // Optional
  // seg.setModelType (pcl::SACMODEL_PLANE);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setDistanceThreshold (100);
  // seg.setInputCloud (cloudPtr_lowerZ);
  // seg.segment (*inliers_seg, *coefficients);
  // AWARN << "Inliners: " << inliers_seg->indices.size() << "/" << cloudPtr_lowerZ->points.size() << " " 
  //   << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " "
  //                             << coefficients->values[2] << " " << coefficients->values[3];

  // Remove Inliers
  //pcl::ExtractIndices<VPoint> extract_seg; // Create the filtering object
  //extract_seg.setInputCloud (cloudPtr_lowerZ_); // Extract the inliers
  //extract_seg.setIndices (inliers_seg);
  //extract_seg.setNegative (true);       // Remove all inliers
  //extract_seg.filter (*cloudPtr_lowerZ_); // All points but inliers
  // Removel Ground Plane by 1: Indices and Extraction 2: Condition @ http://pointclouds.org/documentation/tutorials/remove_outliers.php
  // for (int i = 0; i < filtered_cloud_all_ptr_->points.size(); i++)
  // {
  //   if (inliers_seg->indices.size() == 0) {
  //     filtered_cloud_indices_ptr_->indices.push_back(i);
  //     continue;
  //   }
  //   float norm = sqrt(coefficients->values[0] * coefficients->values[0] + \
  //                     coefficients->values[1] * coefficients->values[1] + \
  //                     coefficients->values[2] * coefficients->values[2] );
  //   float dist = (coefficients->values[0] * filtered_cloud_all_ptr_->points[i].x + \
  //                 coefficients->values[1] * filtered_cloud_all_ptr_->points[i].y + \
  //                 coefficients->values[2] * filtered_cloud_all_ptr_->points[i].z + \
  //                 coefficients->values[3]) / norm;
  //   if (dist > dist_gplane_) {
  //     filtered_cloud_indices_ptr_->indices.push_back(i);
  //   }
  // }

  return;  
}

}  // namespace apollo
