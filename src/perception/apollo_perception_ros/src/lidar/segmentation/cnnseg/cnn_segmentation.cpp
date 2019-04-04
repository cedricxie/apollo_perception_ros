#include "lidar/segmentation/cnnseg/cnn_segmentation.h"

namespace apollo_perception_standalone {

bool CNNSegmentation::Init(std::string &file_path_cnn_segmentation_config) {
  std::string file_path_cnn_segmentation_config_ = 
          file_path_cnn_segmentation_config + "/config/cnn_segmentation_config.pb.txt";
  if (!util::GetProtoFromFile(file_path_cnn_segmentation_config_, &config_)) {
    AERROR << "GetProtoFromFile FAILED!";
    return false;
  }
  AINFO << "GetProtoFromFile SUCCESSFULLY from path: " << file_path_cnn_segmentation_config_;

  // get file paths from passed argument
  std::string file_path_cnn_segmentation_config_file = 
          file_path_cnn_segmentation_config + "/model/cnn_segmentation/cnnseg.conf";
  std::string file_path_cnn_segmentation_proto_file = 
          file_path_cnn_segmentation_config + "/model/cnn_segmentation/deploy.prototxt";
  std::string file_path_cnn_segmentation_weight_file = 
          file_path_cnn_segmentation_config + "/model/cnn_segmentation/deploy.caffemodel";
  if (!util::GetProtoFromFile(file_path_cnn_segmentation_config_file, &cnnseg_param_)) {
    AERROR << "GetProtoFromFile FAILED!";
    return false;
  }
  AINFO << "GetProtoFromFile SUCCESSFULLY from path: " << file_path_cnn_segmentation_config_file;

  /// set parameters
  auto network_param = cnnseg_param_.network_param();
  auto feature_param = cnnseg_param_.feature_param();

  if (feature_param.has_point_cloud_range()) {
    range_ = static_cast<float>(feature_param.point_cloud_range());
  } else {
    range_ = 60.0;
  }
  if (feature_param.has_width()) {
    width_ = static_cast<int>(feature_param.width());
  } else {
    width_ = 640;
  }
  if (feature_param.has_height()) {
    height_ = static_cast<int>(feature_param.height());
  } else {
    height_ = 640;
  }

  AINFO << "using Caffe GPU mode";
  int gpu_id = 0;
  caffe::Caffe::SetDevice(gpu_id);
  caffe::Caffe::set_mode(caffe::Caffe::GPU);
  caffe::Caffe::DeviceQuery();

  caffe_net_.reset(new caffe::Net<float>(file_path_cnn_segmentation_proto_file, caffe::TEST));
  caffe_net_->CopyTrainedLayersFrom(file_path_cnn_segmentation_weight_file);

  /// set related Caffe blobs
  // center offset prediction
  std::string instance_pt_blob_name = network_param.has_instance_pt_blob()
                                          ? network_param.instance_pt_blob()
                                          : "instance_pt";
  instance_pt_blob_ = caffe_net_->blob_by_name(instance_pt_blob_name);
  if (instance_pt_blob_ == nullptr) {
    std::cout << "`" << instance_pt_blob_name << "` not exists!" << std::endl;
  }

  // objectness prediction
  std::string category_pt_blob_name = network_param.has_category_pt_blob()
                                          ? network_param.category_pt_blob()
                                          : "category_score";
  category_pt_blob_ = caffe_net_->blob_by_name(category_pt_blob_name);
  if (category_pt_blob_ == nullptr) {
    std::cout << "`" << category_pt_blob_name << "` not exists!" << std::endl;
  }

  // positiveness (foreground object probability) prediction
  std::string confidence_pt_blob_name = network_param.has_confidence_pt_blob()
                                            ? network_param.confidence_pt_blob()
                                            : "confidence_score";
  confidence_pt_blob_ = caffe_net_->blob_by_name(confidence_pt_blob_name);
  if (confidence_pt_blob_ == nullptr) {
    std::cout << "`" << confidence_pt_blob_name << "` not exists!" << std::endl;
  }

  // object height prediction
  std::string height_pt_blob_name = network_param.has_height_pt_blob()
                                        ? network_param.height_pt_blob()
                                        : "height_pt";
  height_pt_blob_ = caffe_net_->blob_by_name(height_pt_blob_name);
  if (height_pt_blob_ == nullptr) {
    std::cout << "`" << height_pt_blob_name << "` not exists!" << std::endl;
  }

  // raw feature data
  std::string feature_blob_name =
      network_param.has_feature_blob() ? network_param.feature_blob() : "data";
  feature_blob_ = caffe_net_->blob_by_name(feature_blob_name);
  if (feature_blob_ == nullptr) {
    std::cout << "`" << feature_blob_name << "` not exists!" << std::endl;
  }

  // class prediction
  std::string class_pt_blob_name = network_param.has_class_pt_blob()
                                       ? network_param.class_pt_blob()
                                       : "class_score";
  class_pt_blob_ = caffe_net_->blob_by_name(class_pt_blob_name);
  if (class_pt_blob_ == nullptr) {
    std::cout << "`" << class_pt_blob_name << "` not exists!" << std::endl;
  }

  feature_generator_.reset(new cnnseg::FeatureGenerator<float>());
  if (!feature_generator_->Init(feature_param, feature_blob_.get())) {
    AERROR << "Fail to Init feature generator for CNNSegmentation";
    return false;
  }
  AINFO << "Successfully initialized feature generator for CNNSegmentation";
  
  cluster2d_.reset(new cnnseg::Cluster2D());
  if (!cluster2d_->Init(height_, width_, range_)) {
    AERROR << "Fail to Init cluster2d for CNNSegmentation";
  }
  AINFO << "Successfully initialized cluster2d for CNNSegmentation";

  return true;
}

bool CNNSegmentation::Segment(pcl_util::PointCloudPtr pc_ptr,
                              const pcl_util::PointIndices& valid_indices,
                              const SegmentationOptions& options,
                              std::vector<std::shared_ptr<Object>>* objects) {
  objects->clear();
  int num_pts = static_cast<int>(pc_ptr->points.size());
  if (num_pts == 0) {
    std::cout << "None of input points, return directly." << std::endl;
    return true;
  }

  use_full_cloud_ =
      (cnnseg_param_.has_use_full_cloud() ? cnnseg_param_.use_full_cloud()
                                          : false) &&
      (options.origin_cloud != nullptr);
  float filter_thresh = static_cast<float>(cnnseg_param_.filter_thresh());
  float enable_filter_thresh =
      static_cast<float>(cnnseg_param_.enable_filter_thresh());

  // generate raw features
  if (use_full_cloud_) {
    feature_generator_->Generate(options.origin_cloud);
  } else {
    feature_generator_->Generate(pc_ptr);
  }
 
  // network forward process
  caffe::Caffe::set_mode(caffe::Caffe::GPU);

  caffe_net_->Forward();

  // clutser points and construct segments/objects
  float objectness_thresh = cnnseg_param_.has_objectness_thresh()
                                ? cnnseg_param_.objectness_thresh()
                                : 0.5;
  bool use_all_grids_for_clustering =
      cnnseg_param_.has_use_all_grids_for_clustering()
          ? cnnseg_param_.use_all_grids_for_clustering()
          : false;
  cluster2d_->Cluster(*category_pt_blob_, *instance_pt_blob_, pc_ptr,
                      valid_indices, objectness_thresh,
                      use_all_grids_for_clustering);

  caffe::Blob<float>* input_data_blob = feature_blob_.get();
  const float* input_data = input_data_blob->cpu_data();
  const float* input_count_data = input_data + input_data_blob->offset(0, 2);

  cluster2d_->Filter(*confidence_pt_blob_, *height_pt_blob_, input_count_data,
                     filter_thresh, enable_filter_thresh);

  cluster2d_->Classify(*class_pt_blob_);

  float confidence_thresh = cnnseg_param_.has_confidence_thresh()
                                ? cnnseg_param_.confidence_thresh()
                                : 0.1;
  float height_thresh =
      cnnseg_param_.has_height_thresh() ? cnnseg_param_.height_thresh() : 0.5;
  int min_pts_num = cnnseg_param_.has_min_pts_num()
                        ? static_cast<int>(cnnseg_param_.min_pts_num())
                        : 3;
  cluster2d_->GetObjects(confidence_thresh, height_thresh, min_pts_num, objects,
                         input_count_data);
  

  return true;
}

} // namespace apollo_perception_standalone