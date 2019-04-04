#ifndef _APOLLO_PERCEPTION_STANDALONE_LIDAR_CNN_SEGMENTATION_H_
#define _APOLLO_PERCEPTION_STANDALONE_LIDAR_CNN_SEGMENTATION_H_

#include <string>
#include <vector>
#include <memory>

#include "caffe/caffe.hpp"

#include "util/file.h"
#include "util/log.h"

#include "common/object.h"
#include "common/pcl_types.h"

#include "lidar/proto/cnnseg.pb.h"
#include "lidar/proto/cnn_segmentation_config.pb.h"

#include "lidar/segmentation/cnnseg/feature_generator.h"
#include "lidar/segmentation/cnnseg/cluster2d.h"

namespace apollo_perception_standalone {

struct SegmentationOptions {
  // original point cloud without ROI filtering
  pcl_util::PointCloudPtr origin_cloud;
  // indices of roi-filtered cloud in original cloud if enabled
  pcl_util::PointIndicesPtr roi_cloud_indices;
  // indices of non-ground points in original clound if enabled
  pcl_util::PointIndicesPtr non_ground_indices;
};

class CNNSegmentation {
public:
    CNNSegmentation() {}
    ~CNNSegmentation() {}

    bool Init(std::string &file_path_cnn_segmentation_config);

    bool Segment(pcl_util::PointCloudPtr pc_ptr,
               const pcl_util::PointIndices& valid_indices,
               const SegmentationOptions& options,
               std::vector<std::shared_ptr<Object>>* objects);

    float range() const { return range_; }
    int width() const { return width_; }
    int height() const { return height_; }

private:

    // use all points of cloud to compute features
    bool use_full_cloud_ = true;

    // range of bird-view field (for each side)
    float range_ = 0.0;
    // number of cells in bird-view width
    int width_ = 0;
    // number of cells in bird-view height
    int height_ = 0;

    // paramters of CNNSegmentation
    cnnseg::CNNSegParam cnnseg_param_;

    // config file
    cnn_segmentation_config::ModelConfigs config_;

    // Caffe network object
    std::shared_ptr<caffe::Net<float>> caffe_net_;

    // bird-view raw feature generator
    std::shared_ptr<cnnseg::FeatureGenerator<float>> feature_generator_;
    
    // clustering model for post-processing
    std::shared_ptr<cnnseg::Cluster2D> cluster2d_;

    // center offset prediction
    boost::shared_ptr<caffe::Blob<float>> instance_pt_blob_;
    // objectness prediction
    boost::shared_ptr<caffe::Blob<float>> category_pt_blob_;
    // fg probability prediction
    boost::shared_ptr<caffe::Blob<float>> confidence_pt_blob_;
    // object height prediction
    boost::shared_ptr<caffe::Blob<float>> height_pt_blob_;
    // raw features to be input into network
    boost::shared_ptr<caffe::Blob<float>> feature_blob_;
    // class prediction
    boost::shared_ptr<caffe::Blob<float>> class_pt_blob_;

};

} // namespace apollo_perception_standalone
#endif