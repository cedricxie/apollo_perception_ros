#include"perception_yx/lidar_process.h"

namespace apollo_perception_standalone {
using Eigen::Affine3d;
using Eigen::Matrix4d;
using pcl_util::Point;
using pcl_util::PointCloud;
using pcl_util::PointCloudPtr;
using pcl_util::PointD;
using pcl_util::PointIndices;
using pcl_util::PointIndicesPtr;

bool LidarProcess::Init(std::string &path_config_folder, bool log_lidar) {
  // ----------------------------------------------------------------- //
  // ------------- Initialization -------------------------------------//
  // ----------------------------------------------------------------- //
  log_lidar_ = log_lidar;
  
  // roi filter
  std::string file_path_roi_filter_config = 
        path_config_folder + "/config/roi_filter_config.pb.txt";
  roi_filter_ = std::make_unique<apollo_perception_standalone::ROIFilter>();
  if (!roi_filter_->Init(file_path_roi_filter_config)) {
    AERROR << "Init roi filter failed";
  }
  AINFO << "Init successfully, roi filter";
  // segmentor
  std::string file_path_cnn_segmentation_config = path_config_folder;
        // path_config_folder +  "/config/cnn_segmentation_config.pb.txt";
  segmentor_ = std::make_unique<apollo_perception_standalone::CNNSegmentation>();
  if (!segmentor_->Init(file_path_cnn_segmentation_config)) {
    AERROR << "Init segmentor failed";
  }
  AINFO << "Init successfully, segmentor";
  // object filter
  std::string file_path_low_object_filter_config = 
        path_config_folder + "/config/low_object_filter_config.pb.txt";
  object_filter_ = std::make_unique<apollo_perception_standalone::LowObjectFilter>();
  if (!object_filter_->Init(file_path_low_object_filter_config)) {
    AERROR << "Init object filter failed";
  }
  AINFO << "Init successfully, object filter";
  // object builder
  object_builder_ = std::make_unique<apollo_perception_standalone::MinBoxObjectBuilder>();
  if (!object_builder_->Init()) {
    AERROR << "Init object builder failed";
  }
  AINFO << "Init successfully, object builder";
  // tracker
  std::string file_path_hm_tracker_config = 
        path_config_folder + "/config/tracker_config.pb.txt";
  tracker_ = std::make_unique<apollo_perception_standalone::HmObjectTracker>();
  if (!tracker_->Init(file_path_hm_tracker_config)) {
    AERROR << "Failed to Init tracker: " << tracker_->name();
    return false;
  }
  AINFO << "Init successfully, tracker: " << tracker_->name();

  if (!LidarFrameSupplement::state_vars.initialized_) {
    // TODO: check values for Lidar state_vas
    // LidarFrameSupplement::state_vars.process_noise *= 10;
    // LidarFrameSupplement::state_vars.trans_matrix.block(0, 0, 1, 4) << 1.0f,
    //     0.0f, 0.33f, 0.0f;
    // LidarFrameSupplement::state_vars.trans_matrix.block(1, 0, 1, 4) << 0.0f,
    //     1.0f, 0.0f, 0.33f;
    ADEBUG << "state trans matrix in LidarFrameSupplement is \n"
           << LidarFrameSupplement::state_vars.trans_matrix << std::endl;
    LidarFrameSupplement::state_vars.initialized_ = true;
  }

  AINFO << "Initialized successfully,lidar process ";
  inited_ = true;
  
  return true;
}

bool LidarProcess::Process(const pcl_util::VPointCloudPtr &cloud_ptr,
                           pcl_util::VPointCloudPtr &filtered_cloud_ptr,
                           pcl_util::VPointCloudPtr &filtered_cloud_ground_ptr) {
  objects_.clear();
  timestamp_ = pcl_conversions::fromPCL(cloud_ptr->header.stamp).toSec();

  // get velodyne2world transfrom; set fixed pose in navigation mode
  // Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  // std::shared_ptr<Eigen::Matrix4d> lidar_trans = std::make_shared<Eigen::Matrix4d>(pose);
  std::shared_ptr<Matrix4d> lidar_trans = std::make_shared<Matrix4d>();
  if (!GetSensorTrans(timestamp_, lidar_trans.get())) {
    AERROR << "failed to get trans at timestamp: " << timestamp_;
    return false;
  }
  ADEBUG << "get trans pose succ.";

  if (!Process(timestamp_, cloud_ptr, lidar_trans, filtered_cloud_ptr, filtered_cloud_ground_ptr)) {
    AERROR << "faile to process lidar scan at timestamp: " << timestamp_;
    return false;
  }
  return true;
}

bool LidarProcess::Process(const double time_stamp, 
                           const pcl_util::VPointCloudPtr &cloud_ptr,
                           std::shared_ptr<Matrix4d> lidar_trans,
                           pcl_util::VPointCloudPtr &filtered_cloud_ptr,
                           pcl_util::VPointCloudPtr &filtered_cloud_ground_ptr) {
  // fix intensity in apollo demo 2.0 bag
  for (size_t i = 0; i < cloud_ptr->points.size(); i++) {
    // AWARN_IF(i % 10000 == 0) << cloud_ptr->points[i].x << " "
    //   << cloud_ptr->points[i].y << " "
    //   << cloud_ptr->points[i].z << " "
    //   << cloud_ptr->points[i].intensity;
    cloud_ptr->points[i].intensity = 1.0;
  }
  // ----------------------------------------------------------------- //
  // ------------- ROI Filter -----------------------------------------//
  // ----------------------------------------------------------------- //
  pcl_util::PointIndicesPtr filtered_cloud_indices_ptr(new pcl::PointIndices());
  pcl_util::VPointCloudPtr filtered_cloud_all_ptr(new pcl_util::VPointCloud());
  apollo_perception_standalone::ROIFilterOptions roi_filter_options;
  roi_filter_->Filter(roi_filter_options, cloud_ptr, filtered_cloud_all_ptr, filtered_cloud_indices_ptr);
  // extract ground plane points
  pcl::ExtractIndices<pcl_util::VPoint> extract_remv;
  extract_remv.setInputCloud(filtered_cloud_all_ptr);
  extract_remv.setIndices(filtered_cloud_indices_ptr);
  extract_remv.setNegative(true);    // get all but inliers
  extract_remv.filter(*filtered_cloud_ground_ptr); // ground plane
  // extract all but ground plane points
  extract_remv.setInputCloud(filtered_cloud_all_ptr);
  extract_remv.setIndices(filtered_cloud_indices_ptr);
  extract_remv.setNegative(false); // get inliers
  extract_remv.filter(*filtered_cloud_ptr); // all points but ground plane
  // AWARN_IF(log_lidar_) << " filtered_cloud_ground_ptr size: " << filtered_cloud_ground_ptr->points.size()
  //   << " filtered_cloud_ptr size: " << filtered_cloud_ptr->points.size() ;
  // AWANR_IF(log_lidar_) << "RoI Filter done...";
  // ----------------------------------------------------------------- //
  // ------------- Convert Point Cloud and Set Pose -------------------//
  // ----------------------------------------------------------------- //
  apollo_perception_standalone::pcl_util::PointCloudPtr cloud_conv_ptr(new apollo_perception_standalone::pcl_util::PointCloud);
  cloud_conv_ptr->points.resize(filtered_cloud_ptr->size());
  for (size_t i = 0; i < filtered_cloud_ptr->points.size(); i++) {
      cloud_conv_ptr->points[i].x = filtered_cloud_ptr->points[i].x;
      cloud_conv_ptr->points[i].y = filtered_cloud_ptr->points[i].y;
      cloud_conv_ptr->points[i].z = filtered_cloud_ptr->points[i].z;
      cloud_conv_ptr->points[i].intensity = filtered_cloud_ptr->points[i].intensity;
  }
  // ----------------------------------------------------------------- //
  // ------------- CNN Segmentation -----------------------------------//
  // ----------------------------------------------------------------- //
  std::vector<std::shared_ptr<apollo_perception_standalone::Object>> objects; // detected objects
  apollo_perception_standalone::SegmentationOptions segmentation_options;
  segmentation_options.origin_cloud = cloud_conv_ptr;
  apollo_perception_standalone::pcl_util::PointIndices non_ground_indices;
  non_ground_indices.indices.resize(cloud_conv_ptr->points.size());
  std::iota(non_ground_indices.indices.begin(), non_ground_indices.indices.end(), 0);
  segmentor_->Segment(cloud_conv_ptr, non_ground_indices, segmentation_options, &objects);
  AWARN_IF(log_lidar_) << "call segmentation succ. The num of objects is: " << objects.size() << std::endl;
  // sum point clouds of objects
  apollo_perception_standalone::pcl_util::PointCloudPtr cloud_objects(new apollo_perception_standalone::pcl_util::PointCloud);
  for (auto &object : objects) {
    *cloud_objects += *(object->cloud);
  }
  // AWANR_IF(log_lidar_) << "CNN Seg done...";
  // ----------------------------------------------------------------- //
  // ------------- Object Builder and Filter --------------------------//
  // ----------------------------------------------------------------- //
  apollo_perception_standalone::ObjectFilterOptions object_filter_options;
  object_filter_options.velodyne_trans.reset(new Eigen::Matrix4d);
  object_filter_options.velodyne_trans = lidar_trans;
  // object_filter_options.hdmap_struct_ptr = hdmap;
  if (!object_filter_->Filter(object_filter_options, &objects)) {
    //AERROR << "failed to call object filter.";
    return false;
  }
  //std::cout << "call object filter succ. The num of objects is: "
  //    << objects.size() << std::endl;
  if (object_builder_ != nullptr) {
    apollo_perception_standalone::ObjectBuilderOptions object_builder_options;
    if (!object_builder_->Build(object_builder_options, &objects)) {
      //AERROR << "failed to call object builder.";
      return false;
    }
  }
  // AWANR_IF(log_lidar_) << "Object Builder and Filter done...";
  // ----------------------------------------------------------------- //
  // ------------- HM Tracker -----------------------------------------//
  // ----------------------------------------------------------------- //
  if (tracker_ != nullptr) {
    apollo_perception_standalone::TrackerOptions tracker_options;
    tracker_options.velodyne_trans = lidar_trans;
    //tracker_options.hdmap = hdmap;
    //tracker_options.hdmap_input = hdmap_input_;
    if (!tracker_->Track(objects, time_stamp, tracker_options, &objects_, log_lidar_)) {
      AERROR << "failed to call tracker.";
      return false;
    }
  }
  // AWANR_IF(log_lidar_) << "HM Tracker done...";
  
  return true;
}

void LidarProcess::TransPointCloudToPCL(const sensor_msgs::PointCloud2& in_msg,
                                        PointCloudPtr* out_cloud) {
  // transform from ros to pcl
  pcl::PointCloud<pcl_util::PointXYZIT> in_cloud;
  pcl::fromROSMsg(in_msg, in_cloud);
  // transform from xyzit to xyzi
  PointCloudPtr& cloud = *out_cloud;
  cloud->header = in_cloud.header;
  cloud->width = in_cloud.width;
  cloud->height = in_cloud.height;
  cloud->is_dense = in_cloud.is_dense;
  cloud->sensor_origin_ = in_cloud.sensor_origin_;
  cloud->sensor_orientation_ = in_cloud.sensor_orientation_;
  cloud->points.resize(in_cloud.points.size());
  size_t points_num = 0;
  for (size_t idx = 0; idx < in_cloud.size(); ++idx) {
    pcl_util::PointXYZIT& pt = in_cloud.points[idx];
    if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z) &&
        !std::isnan(pt.intensity)) {
      cloud->points[points_num].x = pt.x;
      cloud->points[points_num].y = pt.y;
      cloud->points[points_num].z = pt.z;
      cloud->points[points_num].intensity = pt.intensity;
      ++points_num;
    }
  }
  cloud->points.resize(points_num);
  return;
}

bool LidarProcess::GetSensorTrans(const double query_time, Matrix4d* trans) {
  if (!trans) {
    AERROR << "failed to get trans, the trans ptr can not be nullptr";
    return false;
  }

  ros::Time query_stamp(query_time);
  // const double kTf2BuffSize = FLAGS_tf2_buff_in_ms / 1000.0;
  std::string err_msg;
  if (!tf2_listener_.canTransform(FLAGS_lidar_tf2_frame_id,
                                FLAGS_lidar_tf2_child_frame_id,
                                query_stamp,
                              //  ros::Time(0),
                                &err_msg)) {
    AERROR << "Cannot transform frame: " << FLAGS_lidar_tf2_frame_id
            << " to frame " << FLAGS_lidar_tf2_child_frame_id
            << " , err: " << err_msg
            << ". Frames: " << tf2_listener_.allFramesAsString();
    return false;
  }

  tf::StampedTransform stamped_transform;
  // geometry_msgs::TransformStamped transform_stamped;
  try {
    tf2_listener_.lookupTransform(FLAGS_lidar_tf2_frame_id, 
      FLAGS_lidar_tf2_child_frame_id, query_stamp, stamped_transform);
  } catch (tf2::TransformException& ex) {
    AERROR << "Exception: " << ex.what();
    return false;
  }
  Affine3d affine_3d;
  tf::transformTFToEigen(stamped_transform, affine_3d);
  *trans = affine_3d.matrix();

  // AWARN_IF(log_lidar_) << "get " << FLAGS_lidar_tf2_frame_id << " to "
  //         << FLAGS_lidar_tf2_child_frame_id << " trans: \n" << *trans;
  return true;

  // *****************
  // Original Function
  // *****************
  //   if (!trans) {
  //     AERROR << "failed to get trans, the trans ptr can not be nullptr";
  //     return false;
  //   }

  //   ros::Time query_stamp(query_time);
  //   const auto& tf2_buffer = AdapterManager::Tf2Buffer();

  //   const double kTf2BuffSize = FLAGS_tf2_buff_in_ms / 1000.0;
  //   std::string err_msg;
  //   if (!tf2_buffer.canTransform(FLAGS_lidar_tf2_frame_id,
  //                                FLAGS_lidar_tf2_child_frame_id, query_stamp,
  //                                ros::Duration(kTf2BuffSize), &err_msg)) {
  //     AERROR << "Cannot transform frame: " << FLAGS_lidar_tf2_frame_id
  //            << " to frame " << FLAGS_lidar_tf2_child_frame_id
  //            << " , err: " << err_msg
  //            << ". Frames: " << tf2_buffer.allFramesAsString();
  //     return false;
  //   }

  //   geometry_msgs::TransformStamped transform_stamped;
  //   try {
  //     transform_stamped = tf2_buffer.lookupTransform(
  //         FLAGS_lidar_tf2_frame_id, FLAGS_lidar_tf2_child_frame_id, query_stamp);
  //   } catch (tf2::TransformException& ex) {
  //     AERROR << "Exception: " << ex.what();
  //     return false;
  //   }
  //   Affine3d affine_3d;
  //   tf::transformMsgToEigen(transform_stamped.transform, affine_3d);
  //   *trans = affine_3d.matrix();

  //   ADEBUG << "get " << FLAGS_lidar_tf2_frame_id << " to "
  //          << FLAGS_lidar_tf2_child_frame_id << " trans: " << *trans;
  //   return true;
}

void LidarProcess::GeneratePbMsg(PerceptionObstacles* obstacles) {
  //   common::Header* header = obstacles->mutable_header();
  //   header->set_lidar_timestamp(timestamp_ * 1e9);  // in ns
  //   header->set_camera_timestamp(0);
  //   header->set_radar_timestamp(0);

  //   obstacles->set_error_code(error_code_);

  //   for (const auto& obj : objects_) {
  //     PerceptionObstacle* obstacle = obstacles->add_perception_obstacle();
  //     obj->Serialize(obstacle);
  //     obstacle->set_timestamp(obstacle->timestamp() * 1000);
  //   }

  //   ADEBUG << "PerceptionObstacles: " << obstacles->ShortDebugString();
}
}