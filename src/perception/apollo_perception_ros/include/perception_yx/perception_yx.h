/* -*- mode: C++ -*- */
/*  Copyright (C) 2010 UT-Austin & Austin Robot Technology,
 *  David Claridge, Michael Quinlan
 * 
 *  License: Modified BSD Software License 
 */


#ifndef _PERCEPTION_YX_H_
#define _PERCEPTION_YX_H_

#include "perception_yx/lidar_process.h"
#include "perception_yx/camera_process.h"

#include "perception_yx/obstacle_fusion.h"

#include "radar/interface/base_radar_detector.h"
#include "radar/modest/modest_radar_detector.h"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/StdVector>
#include "yaml-cpp/yaml.h"
#include <pthread.h>

#include "common/pcl_types.h"

namespace perception_yx {

class CameraCalibration {
 public:
  //  intrinsic
  cv::Mat cameraK;
  cv::Mat cameraD;
  //  extrinsic
  std::vector<double> cameraT;  //  x y z
  std::vector<double> cameraR;  //  q0(w) q1(x) q2(y) q3(z)
};

class PerceptionYX
{
public:

  /** Constructor
   *
   *  @param node NodeHandle of this instance
   *  @param private_nh private NodeHandle of this instance
   */
  PerceptionYX(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~PerceptionYX();

  /** callback to process data input
   *
   *  @param scan vector of input 3D data points
   *  @param stamp time stamp of data
   *  @param frame_id data frame of reference
   */
  void processLidarData(const apollo_perception_standalone::pcl_util::VPointCloudPtr& scan);

  void processImageData(const sensor_msgs::ImageConstPtr& msg, const int cam_idx);

  void processRadarData(const apollo_perception_standalone::ContiRadar &msg);

  int parseCameraCalibration(const std::string contents, double resize_factor);

private:

  static unsigned char CLIPVALUE(int val) {
    // Old method (if)
    val = val < 0 ? 0 : val;
    return val > 255 ? 255 : val;

    // New method (array)
    // return uchar_clipping_table[val + clipping_table_offset];
  }

  /**
   * Conversion from YUV to RGB.
   * The normal conversion matrix is due to Julien (surname unknown):
   *
   * [ R ]   [  1.0   0.0     1.403 ] [ Y ]
   * [ G ] = [  1.0  -0.344  -0.714 ] [ U ]
   * [ B ]   [  1.0   1.770   0.0   ] [ V ]
   *
   * and the firewire one is similar:
   *
   * [ R ]   [  1.0   0.0     0.700 ] [ Y ]
   * [ G ] = [  1.0  -0.198  -0.291 ] [ U ]
   * [ B ]   [  1.0   1.015   0.0   ] [ V ]
   *
   * Corrected by BJT (coriander's transforms RGB->YUV and YUV->RGB
   *                   do not get you back to the same RGB!)
   * [ R ]   [  1.0   0.0     1.136 ] [ Y ]
   * [ G ] = [  1.0  -0.396  -0.578 ] [ U ]
   * [ B ]   [  1.0   2.041   0.002 ] [ V ]
   *
   */
  static void YUV2RGB(const unsigned char y, const unsigned char u,
                      const unsigned char v, unsigned char *r, unsigned char *g,
                      unsigned char *b) {
    const int y2 = (int)y;
    const int u2 = (int)u - 128;
    const int v2 = (int)v - 128;
    // std::cerr << "YUV=("<<y2<<","<<u2<<","<<v2<<")"<<std::endl;

    // This is the normal YUV conversion, but
    // appears to be incorrect for the firewire cameras
    //   int r2 = y2 + ( (v2*91947) >> 16);
    //   int g2 = y2 - ( ((u2*22544) + (v2*46793)) >> 16 );
    //   int b2 = y2 + ( (u2*115999) >> 16);
    // This is an adjusted version (UV spread out a bit)
    int r2 = y2 + ((v2 * 37221) >> 15);
    int g2 = y2 - (((u2 * 12975) + (v2 * 18949)) >> 15);
    int b2 = y2 + ((u2 * 66883) >> 15);
    // std::cerr << "   RGB=("<<r2<<","<<g2<<","<<b2<<")"<<std::endl;

    // Cap the values.
    *r = CLIPVALUE(r2);
    *g = CLIPVALUE(g2);
    *b = CLIPVALUE(b2);
  }

  static void Yuyv2rgb(unsigned char *YUV, unsigned char *RGB, int NumPixels) {
    int i, j;
    unsigned char y0, y1, u, v;
    unsigned char r, g, b;

    for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6) {
      y0 = (unsigned char)YUV[i + 0];
      u = (unsigned char)YUV[i + 1];
      y1 = (unsigned char)YUV[i + 2];
      v = (unsigned char)YUV[i + 3];
      YUV2RGB(y0, u, v, &r, &g, &b);
      RGB[j + 0] = r;
      RGB[j + 1] = g;
      RGB[j + 2] = b;
      YUV2RGB(y1, u, v, &r, &g, &b);
      RGB[j + 3] = r;
      RGB[j + 4] = g;
      RGB[j + 5] = b;
    }
  }

  bool get_project_point(Eigen::Matrix4d v2c,
                         Eigen::Vector3d pc,
                         Eigen::Vector2d* p2d,
                         int camera_id) {
    Eigen::Vector3d pc3d =
        (v2c * Eigen::Vector4d(pc[0], pc[1], pc[2], 1)).head(3);
    if (pc3d[2] < 0) {
      return false;
    }
    Eigen::Vector3d pv = camera_intrinsics_[camera_id].block(0, 0, 3, 3) * (pc3d);
    *p2d = pv.head(2) / pv.z();
    return true;
  }

  void convert_rot(const double array[], double rr[]) {
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << array[0], array[1], array[2]);
    cv::Mat rot;
    cv::Rodrigues(rvec, rot);
    for (int i = 0; i < 9; ++i) {
      rr[i] = rot.at<double>(i);
    }
  }

  void convert_rot_trans(const double array[], double rr[], double tt[]) {
    convert_rot(array, rr);
    for (int i = 0; i < 3; ++i) {
      tt[i] = array[3 + i];
    }
  }

  void convert_cv_vec(const double array[], cv::Mat &rvec, cv::Mat &tvec) {
    rvec = (cv::Mat_<double>(3, 1) << array[0], array[1], array[2]);
    tvec = (cv::Mat_<double>(3, 1) << array[3], array[4], array[5]);
  }

  void convert_camera_matrix(const cv::Mat &camera_matrix, double array[]) {
    assert(!camera_matrix.empty() && camera_matrix.rows == 3 &&
          camera_matrix.cols == 3);
    array[0] = camera_matrix.at<double>(0, 0);
    array[1] = camera_matrix.at<double>(1, 1);
    array[2] = camera_matrix.at<double>(0, 2);
    array[3] = camera_matrix.at<double>(1, 2);
  }

  void convert_camera_matrix(const double array[], cv::Mat &camera_matrix) {
    camera_matrix = (cv::Mat_<double>(3, 3) << array[0], 0.0, array[2], 0.0,
                    array[1], array[3], 0.0, 0.0, 1.0);
  }

  void convert_camera_dist(const double array[], cv::Mat &camera_dist) {
    camera_dist = (cv::Mat_<double>(5, 1) << array[0], array[1], array[2],
                  array[3], array[4]);
  }

  void convert_quater_to_axisd(const std::vector<double> q, double axsid[]) {
    Eigen::Quaternion<double> rotation(q[0], q[1], q[2], q[3]);
    Eigen::AngleAxis<double> aa(rotation);
    Eigen::Vector3d avec = aa.axis() * aa.angle();

    axsid[0] = avec[0];
    axsid[1] = avec[1];
    axsid[2] = avec[2];
  }

  bool convert_inv_extrinsics(const double extrinsics_src[],
                              double extrinsics_inv[]) {
    double rr[9];
    double tt[3];
    convert_rot_trans(extrinsics_src, rr, tt);
    extrinsics_inv[0] = -extrinsics_src[0];
    extrinsics_inv[1] = -extrinsics_src[1];
    extrinsics_inv[2] = -extrinsics_src[2];
    extrinsics_inv[3] = -rr[0] * tt[0] - rr[3] * tt[1] - rr[6] * tt[2];
    extrinsics_inv[4] = -rr[1] * tt[0] - rr[4] * tt[1] - rr[7] * tt[2];
    extrinsics_inv[5] = -rr[2] * tt[0] - rr[5] * tt[1] - rr[8] * tt[2];
  }

  void convert_quater_to_trans(const std::vector<double> q,
                              const std::vector<double> tt, double array[]) {
    convert_quater_to_axisd(q, array);
    array[3] = tt[0];
    array[4] = tt[1];
    array[5] = tt[2];
  }

  void compute_color_map(int color_mode, std::vector<cv::Scalar> &clr_vec) {
    cv::Mat color_map_gray = cv::Mat::zeros(256, 1, CV_8UC1);
    for (int i = 0; i < color_map_gray.rows; ++i) {
      color_map_gray.at<uchar>(i) = 255 - i;
    }
    if (color_mode >= 0) {
      cv::Mat color_map;
      cv::applyColorMap(color_map_gray, color_map, color_mode);
      for (int i = 0; i < color_map.total(); ++i) {
        cv::Vec3b clr = color_map.at<cv::Vec3b>(i);
        clr_vec.push_back(cv::Scalar(clr[0], clr[1], clr[2], 255));
      }
    } else {
      for (int i = 0; i < color_map_gray.total(); ++i) {
        clr_vec.push_back(cv::Scalar(i, i, i, 255));
      }
    }
  }

  template <typename T>
  std::string to_string_with_precision(const T a_value, const int n = 6) {
      std::ostringstream out;
      out.precision(n);
      out << std::fixed << a_value;
      return out.str();
  }

  bool messageToMat(const sensor_msgs::Image &msg,
                    cv::Mat *img);
  
  bool matToMessage(const cv::Mat &img,
                          sensor_msgs::Image *msg);

  bool projectCloudToImage(double timestamp,
                           apollo_perception_standalone::pcl_util::VPointCloud &cloud,
                           const double intrinsic[], const double distortion[],
                           const double extrinsic[], int render_mode,
                           double min_v, double max_v,
                           const std::vector<cv::Scalar> &clr_vec,
                           int point_size, int thinkness, int point_type,
                           cv::Mat &out_image);

  void processPointCloud(const apollo_perception_standalone::pcl_util::VPointCloudPtr &scan);

  void processFusion(
          std::shared_ptr<apollo_perception_standalone::SensorObjects> &in_sensor_objects);

  void publishFusion(std::vector<std::shared_ptr<apollo_perception_standalone::Object>> &fused_objects, 
                      double timestamp);

  bool getSensorTrans(const double query_time, Eigen::Matrix4d* trans, 
                      std::string parent_frame_id, std::string child_frame_id);

  void objToBBox(std::shared_ptr<apollo_perception_standalone::Object> &obj,
                 apollo_perception_standalone::SensorType sensor_type,
                 int cam_idx);

  visualization_msgs::Marker objToMarker(std::shared_ptr<apollo_perception_standalone::Object> &obj,
                                                     std::string &frame_id,
                                                     ros::Time &timestamp,
                                                     std::string &ns,
                                                     int32_t id,
                                                     int32_t type);

  void objToImg(cv::Mat &image, bool show_2d,
                Eigen::Matrix4d pose_c2w, Eigen::Matrix4d pose_c2w_static,
                const std::vector<std::shared_ptr<apollo_perception_standalone::Object>>& objects,
                const apollo_perception_standalone::CameraFrameSupplement& supplement,
                int camera_id);

  // TF listener
  tf::TransformListener tf2_listener_;

  // Sensor control
  bool use_lidar_ = false;
  bool use_cam_long_ = false;
  bool use_cam_short_ = false;
  bool use_radar_ = false;

  // Parameters for logging
  bool log_cam_;
  bool log_radar_;
  bool log_lidar_;
  bool log_fusion_;

  // Frame IDs
  std::string frame_id_cam_;
  std::string frame_id_lidar_;
  std::string frame_id_radar_;
  std::string frame_id_fusion_;
  
  // Parameters for cameras
  CameraCalibration calibs_[2];
  double intrinsic_[2][4];
  double distortion_[2][5];
  double extrinsic_[2][6];
  std::vector<Eigen::Matrix<double, 3, 4>, 
              Eigen::aligned_allocator<Eigen::Matrix<double, 3, 4> > > camera_intrinsics_;
  int render_mode_ = 2; // 0, intensity; 1, height; 2, distance
  int min_v_ = 10, max_v_ = 30;
  int point_size_ = 3;
  int point_type_ = 0; // 0, dot; 1, cross
  int thinkness_ = 3;
  int clr_mode_ = 4;
  std::vector<cv::Scalar> clr_vec_;
  double resize_factor_ = 1.0;

  // Parameters for projection
  bool proj_ptcloud_;
  pthread_mutex_t proj_lock_;

  // Camera images
  sensor_msgs::Image image_long_;
  sensor_msgs::Image image_short_;

  // Point clouds
  apollo_perception_standalone::pcl_util::VPointCloudPtr filtered_cloud_ptr_;
  apollo_perception_standalone::pcl_util::VPointCloudPtr filtered_cloud_objects_ptr_;
  apollo_perception_standalone::pcl_util::VPointCloudPtr filtered_cloud_ground_ptr_;    

  // Markers for visualization
  visualization_msgs::Marker lidar_bbox_list_;               // id: 1
  std::vector<visualization_msgs::Marker> camera_bbox_list_; // id: 10+
  // visualization_msgs::Marker camera_bbox_list_;
  visualization_msgs::Marker radar_bbox_list_;               // id: 20
  visualization_msgs::Marker fusion_bbox_list_;              // id: 30
  visualization_msgs::MarkerArray lidar_velocity_list_;      // id: 100+
  visualization_msgs::MarkerArray radar_velocity_list_;      // id: 200+
  visualization_msgs::MarkerArray fusion_velocity_list_;      // id: 300+

  bool vis_lidar_poly_;
  bool vis_cam_sphere_;
  bool vis_radar_sphere_;

  // Subscriber
  ros::Subscriber lidar_scan_;
  ros::Subscriber camera_long_;
  ros::Subscriber camera_short_;
  ros::Subscriber radar_conti_;

  // Data publisher
  ros::Publisher image_long_publisher_;
  ros::Publisher image_short_publisher_;
  ros::Publisher filtered_cloud_publisher_;
  ros::Publisher filtered_cloud_ground_publisher_;
  ros::Publisher filtered_cloud_objects_publisher_;
  
  // Markers publisher
  ros::Publisher lidar_bbox_publisher_;
  ros::Publisher lidar_velocity_publisher_; 
  ros::Publisher camera_long_bbox_publisher_;
  ros::Publisher camera_short_bbox_publisher_;
  ros::Publisher radar_bbox_publisher_;
  ros::Publisher radar_velocity_publisher_;
  ros::Publisher fusion_bbox_publisher_;
  ros::Publisher fusion_velocity_publisher_;

  // obstacle detection
  std::unique_ptr<apollo_perception_standalone::LidarProcess> lidar_process_;
  std::unique_ptr<apollo_perception_standalone::CameraProcess> camera_long_process_;
  std::unique_ptr<apollo_perception_standalone::CameraProcess> camera_short_process_;
  std::unique_ptr<apollo_perception_standalone::ModestRadarDetector> radar_detector_;

  // tracked objects
  std::shared_ptr<apollo_perception_standalone::SensorObjects> lidar_objects_;
  std::shared_ptr<apollo_perception_standalone::SensorObjects> camera_long_objects_;
  std::shared_ptr<apollo_perception_standalone::SensorObjects> camera_short_objects_;
  std::shared_ptr<apollo_perception_standalone::SensorObjects> radar_objects_;

  // obstacle fusion
  std::mutex fusion_pub_mutex_;
  apollo_perception_standalone::ObstacleFusion obstacle_fusion_;
};

} // namespace velodyne_perception_yx

#endif
