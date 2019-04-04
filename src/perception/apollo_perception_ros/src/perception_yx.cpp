/**
 * @file perception_yx.cpp
 * @author Yuesong Xie (cedric_xie@hotmail.com)
 * @brief ROS class for lidar perception
 * 
 * This class utilizes the perception module from Apollo
 * and output object list.
 * 
 * @version 0.1
 * @date 2018-11-16
 * @copyright Copyright (c) 2018
 * 
 */

#include <perception_yx/perception_yx.h>

namespace perception_yx {

/**
 * @brief Construct a new PerceptionYX object
 * 
 * @param node ros node handle
 * @param priv_nh ros private node handle
 */
PerceptionYX::PerceptionYX(ros::NodeHandle node, ros::NodeHandle priv_nh) {
  // Get parameters using private node handle
  std::string path_config_folder;
  priv_nh.param("path_config_folder", path_config_folder, std::string(""));
  priv_nh.param("resize_factor", resize_factor_, 1.0);
  // sensor control
  priv_nh.param("use_lidar", use_lidar_, false);
  priv_nh.param("use_cam_long", use_cam_long_, false);
  priv_nh.param("use_cam_short", use_cam_short_, false);
  priv_nh.param("use_radar", use_radar_, false);
  // visualization setting
  priv_nh.param("vis_lidar_poly", vis_lidar_poly_, false);
  priv_nh.param("vis_cam_sphere", vis_cam_sphere_, false);
  priv_nh.param("vis_radar_sphere", vis_radar_sphere_, false);
  priv_nh.param("proj_ptcloud", proj_ptcloud_, false);
  // frame id
  priv_nh.param("frame_id_cam", frame_id_cam_, std::string(""));
  priv_nh.param("frame_id_lidar", frame_id_lidar_, std::string(""));
  priv_nh.param("frame_id_radar", frame_id_radar_, std::string(""));
  priv_nh.param("frame_id_fusion", frame_id_fusion_, std::string(""));
  // log setting
  priv_nh.param("log_cam", log_cam_, false);
  priv_nh.param("log_radar", log_radar_, false);
  priv_nh.param("log_lidar", log_lidar_, false);
  priv_nh.param("log_fusion", log_fusion_, false);

  AWARN << "Sensor Lidar: " 
        << (use_lidar_ ? "\033[1;32mEnabled\033[0m" : "\033[1;31mDisabled\033[0m") 
        << ", Logging: " 
        << (log_lidar_ ? "\033[1;32mTrue\033[0m" : "\033[1;31mFalse\033[0m");
  AWARN << "Sensor Radar: " 
        << (use_radar_ ? "\033[1;32mEnabled\033[0m" : "\033[1;31mDisabled\033[0m") 
        << ", Logging: " 
        << (log_radar_ ? "\033[1;32mTrue\033[0m" : "\033[1;31mFalse\033[0m");
  AWARN << "Sensor Camera Long: " 
        << (use_cam_long_ ? "\033[1;32mEnabled\033[0m" : "\033[1;31mDisabled\033[0m") 
        << ", Logging: " 
        << (log_cam_ ? "\033[1;32mTrue\033[0m" : "\033[1;31mFalse\033[0m");
  AWARN << "Sensor Camera Short: " 
        << (use_cam_short_ ? "\033[1;32mEnabled\033[0m" : "\033[1;31mDisabled\033[0m") 
        << ", Logging: " 
        << (log_cam_ ? "\033[1;32mTrue\033[0m" : "\033[1;31mFalse\033[0m");
  AWARN << "Fusion, Logging: " 
        << (log_fusion_ ? "\033[1;32mTrue\033[0m" : "\033[1;31mFalse\033[0m");
  AWARN << "Image Resize Factor: " << "\033[1;33m" << resize_factor_ << "\033[0m";

  // mutex
  pthread_mutex_init(&proj_lock_, NULL);
  // color map
  compute_color_map(clr_mode_, clr_vec_);

  // Read in camera parameters
  std::string file_path_camera_yaml = path_config_folder + "/config/calibration.yaml";
  if (parseCameraCalibration(file_path_camera_yaml, resize_factor_) != 0) {
    AERROR << "failed to Parse camera calibration";
    return;
  }
  
  // Initialize lidar process
  lidar_process_ = std::make_unique<apollo_perception_standalone::LidarProcess>();
  if (!lidar_process_->Init(path_config_folder, log_lidar_)) {
      AERROR << "failed to Init lidar_process.";
      return;
  }

  // Initialize camera process
  camera_long_process_ = std::make_unique<apollo_perception_standalone::CameraProcess>();
  if (!camera_long_process_->Init(0, path_config_folder, log_cam_)) {  // 0: camera_long; 1: camera_short
      AERROR << "failed to Init camera_long_process.";
      return;
  }
  camera_short_process_ = std::make_unique<apollo_perception_standalone::CameraProcess>();
  if (!camera_short_process_->Init(1, path_config_folder, log_cam_)) {  // 0: camera_long; 1: camera_short
      AERROR << "failed to Init camera_short_process.";
      return;
  }

  // Initialize radar detector
  radar_detector_.reset(new apollo_perception_standalone::ModestRadarDetector);
  if (radar_detector_ == nullptr) {
    AERROR << "Failed to get RadarDetector plugin ";
          // << FLAGS_onboard_radar_detector;
    return;
  }
  if (!radar_detector_->Init(path_config_folder, log_radar_)) {
    AERROR << "Failed to initialize RadarDetector ";
          // << FLAGS_onboard_radar_detector;
    return;
  }

  // Initialize fusion
  if (!obstacle_fusion_.Init(path_config_folder, log_fusion_)) {
    AERROR << "Failed to init ObstacleFusion";
    return;
  }

  // Initialize visualization marker vector
  // lidar bbox line list
  // http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html
  // http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines
  lidar_bbox_list_.header.frame_id = frame_id_fusion_;
  lidar_bbox_list_.ns = "lidar bbox line list";
  lidar_bbox_list_.action = visualization_msgs::Marker::ADD;
  lidar_bbox_list_.pose.orientation.w = 1.0;
  lidar_bbox_list_.id = 1;
  lidar_bbox_list_.type = visualization_msgs::Marker::LINE_LIST;
  //lidar_bbox_list_.lifetime = ros::Duration(1.0);
  lidar_bbox_list_.scale.x = 0.1; // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  if (vis_lidar_poly_ == true) {
    lidar_bbox_list_.color.r = 1.0; 
    lidar_bbox_list_.color.g = 0.0; 
    lidar_bbox_list_.color.b = 0.0; 
    lidar_bbox_list_.color.a = 1.0; // red
  }

  // camera bbox list
  camera_bbox_list_.resize(2);
  for (int i = 0; i < 2; i++) {
    camera_bbox_list_[i].header.frame_id = frame_id_fusion_;
    camera_bbox_list_[i].ns = "camera bbox line list";
    camera_bbox_list_[i].action = visualization_msgs::Marker::ADD;
    camera_bbox_list_[i].pose.orientation.w = 1.0;
    camera_bbox_list_[i].id = 10 + i;
    if (vis_cam_sphere_ == true) {
      camera_bbox_list_[i].type = visualization_msgs::Marker::SPHERE_LIST;
      camera_bbox_list_[i].scale.x = 1.0;
      camera_bbox_list_[i].scale.y = 1.0;
      camera_bbox_list_[i].scale.z = 1.0;
    } else {
      camera_bbox_list_[i].type = visualization_msgs::Marker::LINE_LIST;
      camera_bbox_list_[i].scale.x = 0.1;
    }
    //camera_bbox_list_[i].lifetime = ros::Duration(1.0);
  }

  // radar bbox line list
  radar_bbox_list_.header.frame_id = frame_id_fusion_;
  radar_bbox_list_.ns = "radar bbox line list";
  radar_bbox_list_.action = visualization_msgs::Marker::ADD;
  radar_bbox_list_.pose.orientation.w = 1.0;
  radar_bbox_list_.id = 20;
  //radar_bbox_list_.lifetime = ros::Duration(1.0);
  if (vis_radar_sphere_ == true) {
    radar_bbox_list_.type = visualization_msgs::Marker::SPHERE_LIST;
    radar_bbox_list_.scale.x = 1.0;
    radar_bbox_list_.scale.y = 1.0;
    radar_bbox_list_.scale.z = 1.0;
  } else {
    radar_bbox_list_.type = visualization_msgs::Marker::LINE_LIST;
    radar_bbox_list_.scale.x = 0.1;
    radar_bbox_list_.color.r = 0.0; 
    radar_bbox_list_.color.g = 0.0; 
    radar_bbox_list_.color.b = 1.0; // blue 
    radar_bbox_list_.color.a = 1.0;
  }
  
  // fusion bbox line list
  fusion_bbox_list_.header.frame_id = frame_id_fusion_;
  fusion_bbox_list_.ns = "fusion bbox line list";
  fusion_bbox_list_.action = visualization_msgs::Marker::ADD;
  fusion_bbox_list_.pose.orientation.w = 1.0;
  fusion_bbox_list_.id = 30;
  //fusion_bbox_list_.lifetime = ros::Duration(1.0);
  fusion_bbox_list_.type = visualization_msgs::Marker::LINE_LIST;
  fusion_bbox_list_.scale.x = 0.1;

  // Initialze SensorObjects
  lidar_objects_.reset(new apollo_perception_standalone::SensorObjects);
  camera_long_objects_.reset(new apollo_perception_standalone::SensorObjects);
  camera_short_objects_.reset(new apollo_perception_standalone::SensorObjects);
  radar_objects_.reset(new apollo_perception_standalone::SensorObjects);

  // Initialize point cloud pointers
  filtered_cloud_ptr_.reset(new apollo_perception_standalone::pcl_util::VPointCloud);
  filtered_cloud_objects_ptr_.reset(new apollo_perception_standalone::pcl_util::VPointCloud);
  filtered_cloud_ground_ptr_.reset(new apollo_perception_standalone::pcl_util::VPointCloud);
  
  // Publishers: lidar
  filtered_cloud_publisher_ = 
    node.advertise<apollo_perception_standalone::pcl_util::VPointCloud>("perception/output/lidar_filtered",1);
  filtered_cloud_objects_publisher_ = 
    node.advertise<apollo_perception_standalone::pcl_util::VPointCloud>("perception/output/lidar_filtered_objects",1);
  filtered_cloud_ground_publisher_ = 
    node.advertise<apollo_perception_standalone::pcl_util::VPointCloud>("perception/output/lidar_filtered_ground",1);
  lidar_bbox_publisher_ = 
    node.advertise<visualization_msgs::Marker>("perception/output/lidar_bbox_marker", 1);
  lidar_velocity_publisher_ = 
    node.advertise<visualization_msgs::MarkerArray>("perception/output/lidar_velocity_marker", 1);

  // Publishers: cameras
  image_long_publisher_ = 
    node.advertise<sensor_msgs::Image>("perception/output/cam_long", 1);
  image_short_publisher_ = 
    node.advertise<sensor_msgs::Image>("perception/output/cam_short", 1);
  camera_long_bbox_publisher_ = 
    node.advertise<visualization_msgs::Marker>("perception/output/cam_long_bbox_marker", 1);
  camera_short_bbox_publisher_ = 
    node.advertise<visualization_msgs::Marker>("perception/output/cam_short_bbox_marker", 1);

  // Pulisher: radar
  radar_bbox_publisher_ = 
    node.advertise<visualization_msgs::Marker>("perception/output/radar_bbox_marker", 1);
  radar_velocity_publisher_ = 
    node.advertise<visualization_msgs::MarkerArray>("perception/output/radar_velocity_marker", 1);

  // Publisher: fusion
  fusion_bbox_publisher_ = 
    node.advertise<visualization_msgs::Marker>("perception/output/fusion_bbox_marker", 1);
  fusion_velocity_publisher_ = 
    node.advertise<visualization_msgs::MarkerArray>("perception/output/fusion_velocity_marker", 1);
  
  // Subscribers
  // http://docs.ros.org/api/roscpp/html/classros_1_1TransportHints.html#a03191a9987162fca0ae2c81fa79fcde9
  std::string topic_name_point_cloud_in = "/apollo/sensor/velodyne64/compensator/PointCloud2";
  lidar_scan_ = node.subscribe(topic_name_point_cloud_in, 1,
                                  &PerceptionYX::processLidarData, this,
                                  ros::TransportHints().reliable().tcpNoDelay(true));
  std::string topic_name_image_long_in = "/apollo/sensor/camera/traffic/image_long";
  const int cam_idx_long = 0;
  camera_long_ = node.subscribe<sensor_msgs::Image>(topic_name_image_long_in, 2,
                                  boost::bind(&PerceptionYX::processImageData, this, _1, cam_idx_long));
  std::string topic_name_image_short_in = "/apollo/sensor/camera/traffic/image_short";
  const int cam_idx_short = 1;
  camera_short_ = node.subscribe<sensor_msgs::Image>(topic_name_image_short_in, 2,
                                  boost::bind(&PerceptionYX::processImageData, this, _1, cam_idx_short));
  // std::string topic_name_radar_conti_in = "/apollo/sensor/conti_radar";
  // radar_conti_ = node.subscribe(topic_name_radar_conti_in, 10,
  //                                     &PerceptionYX::processRadarData, this, 
  //                                     ros::TransportHints().reliable().tcpNoDelay(true));
}

PerceptionYX::~PerceptionYX() {}

int PerceptionYX::parseCameraCalibration(const std::string contents, double resize_factor = 1.0) {
  // std::cout << "Parse Camera Calibration at " << contents << std::endl;
  if (contents.empty()) {
    std::cout << "string is empty" << std::endl;
    return -1;
  }
  YAML::Node yn = YAML::LoadFile(contents);
  std::string cameraId;
  for (int id = 0; id < 2; ++id) {
    cameraId = std::to_string(id) ;
    // cameraId = boost::lexical_cast<std::string>(id);
    // std::cout << "cameraID: " << cameraId << std::endl;
    cv::Mat intrinsicK, intrinsicD;
    // get intrinsicK
    if (yn[cameraId]["K"].IsDefined()) {
      intrinsicK = cv::Mat::zeros(3, 3, CV_64FC1);
      for (int i = 0; i < yn[cameraId]["K"].size(); ++i) {
        intrinsicK.at<double>(i) = yn[cameraId]["K"][i].as<double>();
        if (i < 6) {
          intrinsicK.at<double>(i) *= resize_factor;  // change in K due to resize
        }
      }
      calibs_[id].cameraK = intrinsicK;
      Eigen::Matrix<double, 3, 4> camera_int;
      camera_int(0, 0) = yn[cameraId]["K"][0].as<double>();
      camera_int(0, 1) = yn[cameraId]["K"][1].as<double>();
      camera_int(0, 2) = yn[cameraId]["K"][2].as<double>();
      camera_int(0, 3) = 0.0;
      camera_int(1, 0) = yn[cameraId]["K"][3].as<double>();
      camera_int(1, 1) = yn[cameraId]["K"][4].as<double>();
      camera_int(1, 2) = yn[cameraId]["K"][5].as<double>();
      camera_int(1, 3) = 0.0;
      camera_int(2, 0) = yn[cameraId]["K"][6].as<double>();
      camera_int(2, 1) = yn[cameraId]["K"][7].as<double>();
      camera_int(2, 2) = yn[cameraId]["K"][8].as<double>();
      camera_int(2, 3) = 0.0;
      camera_intrinsics_.push_back(camera_int);
    } else {
      printf("invalid intrinsicFile content\n");
      return -1;
    }
    // GET intrinsicD
    if (yn[cameraId]["D"].IsDefined()) {
      // std::cout<<"type: " << yn[cameraId]["D"].Type()<<std::endl;
      intrinsicD = cv::Mat::zeros(yn[cameraId]["D"].size(), 1, CV_64FC1);
      for (int i = 0; i < yn[cameraId]["D"].size(); ++i) {
        intrinsicD.at<double>(i) = yn[cameraId]["D"][i].as<double>();
      }
      calibs_[id].cameraD = intrinsicD;
    } else {
      printf("invalid intrinsicFile content\n");
      return -1;
    }
    // get camera
    if (yn[cameraId]["transform"]["rotation"]["x"].IsDefined() &&
        yn[cameraId]["transform"]["rotation"]["y"].IsDefined() &&
        yn[cameraId]["transform"]["rotation"]["z"].IsDefined() &&
        yn[cameraId]["transform"]["rotation"]["w"].IsDefined()) {
      calibs_[id].cameraR.push_back(
          yn[cameraId]["transform"]["rotation"]["w"].as<double>());
      calibs_[id].cameraR.push_back(
          yn[cameraId]["transform"]["rotation"]["x"].as<double>());
      calibs_[id].cameraR.push_back(
          yn[cameraId]["transform"]["rotation"]["y"].as<double>());
      calibs_[id].cameraR.push_back(
          yn[cameraId]["transform"]["rotation"]["z"].as<double>());
    } else {
      printf("invalid intrinsicFile content\n");
      return -1;
    }
    // get cameraR
    if (yn[cameraId]["transform"]["translation"]["x"].IsDefined() &&
        yn[cameraId]["transform"]["translation"]["y"].IsDefined() &&
        yn[cameraId]["transform"]["translation"]["z"].IsDefined()) {
      calibs_[id].cameraT.push_back(
          yn[cameraId]["transform"]["translation"]["x"].as<double>());
      calibs_[id].cameraT.push_back(
          yn[cameraId]["transform"]["translation"]["y"].as<double>());
      calibs_[id].cameraT.push_back(
          yn[cameraId]["transform"]["translation"]["z"].as<double>());
    } else {
      printf("invalid intrinsicFile content\n");
      return -1;
    }
  }

  for (int i = 0; i < 2; ++i) {
    /* code */
    convert_camera_matrix(calibs_[i].cameraK, intrinsic_[i]);

    convert_quater_to_trans(calibs_[i].cameraR, calibs_[i].cameraT,
                            extrinsic_[i]);

    convert_inv_extrinsics(extrinsic_[i], extrinsic_[i]);

    for (int j = 0; j < 5; ++j) {
      distortion_[i][j] = calibs_[i].cameraD.at<double>(j);
    }
  }

  return 0;
}

void PerceptionYX::objToImg(cv::Mat &image, bool show_2d,
              Eigen::Matrix4d pose_c2w, Eigen::Matrix4d pose_c2w_static,
              const std::vector<std::shared_ptr<apollo_perception_standalone::Object>>& objects,
              const apollo_perception_standalone::CameraFrameSupplement& supplement,
              int camera_id) {
  int image_width = image.cols;
  int image_height = image.rows;
  Eigen::Matrix4d v2c = pose_c2w.inverse();
  Eigen::Matrix4d v2c_static = pose_c2w_static.inverse();
  cv::Scalar line_color(255,255,10);
  if (camera_id == 3) {
    line_color = cv::Scalar(255, 127, 80);
  }
  int line_thickness = 5;
  for (auto obj : objects) {
    if (obj->camera_supplement != nullptr && show_2d) {
      auto upper_left_pt = obj->camera_supplement->upper_left;
      auto lower_right_pt = obj->camera_supplement->lower_right;
      cv::Point pt_ul(upper_left_pt(0), upper_left_pt(1));
      cv::Point pt_ll(lower_right_pt(0), upper_left_pt(1));
      cv::Point pt_lr(lower_right_pt(0), lower_right_pt(1));
      cv::Point pt_ur(upper_left_pt(0), lower_right_pt(1));
      cv::line(image, pt_ul, pt_ll, line_color, line_thickness);
      cv::line(image, pt_ll, pt_lr, line_color, line_thickness);
      cv::line(image, pt_lr, pt_ur, line_color, line_thickness);
      cv::line(image, pt_ur, pt_ul, line_color, line_thickness);
    } else if (obj->camera_supplement != nullptr && !show_2d) {
      // Eigen::Vector3d center = obj->center;
      // Eigen::Vector2d center2d;
      // get_project_point(v2c, center, &center2d, camera_id);
      std::vector<Eigen::Vector2d> points;
      points.resize(8);
      for (int i = 0; i < 8; i++) {
        points[i].x() = obj->camera_supplement->pts8[i * 2 + 0];
        points[i].y() = obj->camera_supplement->pts8[i * 2 + 1];
      }
      cv::Point p1(points[0][0], points[0][1]);
      cv::Point p2(points[1][0], points[1][1]);
      cv::Point p3(points[2][0], points[2][1]);
      cv::Point p4(points[3][0], points[3][1]);
      cv::Point p5(points[4][0], points[4][1]);
      cv::Point p6(points[5][0], points[5][1]);
      cv::Point p7(points[6][0], points[6][1]);
      cv::Point p8(points[7][0], points[7][1]);
      cv::line(image, p1, p2, line_color, line_thickness);
      cv::line(image, p2, p3, line_color, line_thickness);
      cv::line(image, p3, p4, line_color, line_thickness);
      cv::line(image, p4, p1, line_color, line_thickness);
      cv::line(image, p5, p6, line_color, line_thickness);
      cv::line(image, p6, p7, line_color, line_thickness);
      cv::line(image, p7, p8, line_color, line_thickness);
      cv::line(image, p8, p5, line_color, line_thickness);
      cv::line(image, p5, p1, line_color, line_thickness);
      cv::line(image, p3, p7, line_color, line_thickness);
      cv::line(image, p4, p8, line_color, line_thickness);
      cv::line(image, p6, p2, line_color, line_thickness);
      
    } else {
      AERROR << "camera supplement is NULL!";
    }
  }
  return;
}

void PerceptionYX::objToBBox(std::shared_ptr<apollo_perception_standalone::Object> &obj, 
                             apollo_perception_standalone::SensorType sensor_type,
                             int cam_idx = -1) {
  if (vis_lidar_poly_ == true && is_lidar(sensor_type)) {  // show lidar objects with polygon
    double h = obj->height;
    for (int i = 0; i < obj->polygon.points.size(); i++) {
      geometry_msgs::Point p_0;
      p_0.x = obj->polygon.points[i].x;
      p_0.y = obj->polygon.points[i].y;
      p_0.z = obj->polygon.points[i].z;
      geometry_msgs::Point p_1;
      if (i < obj->polygon.points.size() - 1) {
        p_1.x = obj->polygon.points[i + 1].x;
        p_1.y = obj->polygon.points[i + 1].y;
        p_1.z = obj->polygon.points[i + 1].z;
      } else {
        p_1.x = obj->polygon.points[0].x;
        p_1.y = obj->polygon.points[0].y;
        p_1.z = obj->polygon.points[0].z;
      }
      lidar_bbox_list_.points.push_back(p_0); lidar_bbox_list_.points.push_back(p_1);
      geometry_msgs::Point p_0_h = p_0;
      p_0_h.z += h;
      lidar_bbox_list_.points.push_back(p_0); lidar_bbox_list_.points.push_back(p_0_h);
      geometry_msgs::Point p_1_h = p_1;
      p_1_h.z += h;
      lidar_bbox_list_.points.push_back(p_0_h); lidar_bbox_list_.points.push_back(p_1_h); 
    }
  } else if (vis_cam_sphere_ == true && is_camera(sensor_type)) {  // show camera objects as shpere
    geometry_msgs::Point p;
    p.x = obj->center(0);
    p.y = obj->center(1);
    p.z = obj->center(2);
    camera_bbox_list_[cam_idx].points.push_back(p);
    std_msgs::ColorRGBA p_color;
    if (cam_idx == 0) {
      p_color.a = 1.0;  // yellow
      p_color.r = 1.0;
      p_color.g = 1.0;
      p_color.b = 0.0;
    } else if (cam_idx == 1) {
      p_color.a = 1.0;  // orange
      p_color.r = 1.0;
      p_color.g = 0.5;
      p_color.b = 0.3;
    }
    camera_bbox_list_[cam_idx].colors.push_back(p_color);
  } else if (vis_radar_sphere_ == true && is_radar(sensor_type)) { // show radar objects as shpere
    geometry_msgs::Point p;
    p.x = obj->center(0);
    p.y = obj->center(1);
    p.z = obj->center(2);
    radar_bbox_list_.points.push_back(p);
    std_msgs::ColorRGBA p_color;
    if (sensor_type == apollo_perception_standalone::SensorType::BOSCH_MRR) {
      p_color.a = obj->score;
      p_color.r = 0.0;
      p_color.g = 0.0;
      p_color.b = 1.0;         // blue
    } else if (sensor_type == apollo_perception_standalone::SensorType::FUJITSU_ESR) {
      p_color.a = obj->score;  // sky blue
      p_color.r = 0.5;
      p_color.g = 0.8;
      p_color.b = 0.9;
    } else if (sensor_type == apollo_perception_standalone::SensorType::DELPHI_ESR) {
      p_color.a = 1.0;
      p_color.r = 0.0;
      p_color.g = 0.0;
      p_color.b = 1.0;         // blue
    } else if (sensor_type == apollo_perception_standalone::SensorType::DELPHI_SRR) {
      p_color.a = 1.0;  // sky blue
      p_color.r = 0.5;
      p_color.g = 0.8;
      p_color.b = 0.9;
    }
    radar_bbox_list_.colors.push_back(p_color);
  } else {  // visualize as rectangles
    Eigen::Vector3d dir(cos(obj->theta), sin(obj->theta), 0);
    Eigen::Vector3d odir(-dir[1], dir[0], 0);
    Eigen::Vector3d bottom_quad[4];
    double half_l = obj->length / 2;
    double half_w = obj->width / 2;
    double h = obj->height;
    bottom_quad[0] = obj->center - dir * half_l - odir * half_w;
    bottom_quad[1] = obj->center + dir * half_l - odir * half_w;
    bottom_quad[2] = obj->center + dir * half_l + odir * half_w;
    bottom_quad[3] = obj->center - dir * half_l + odir * half_w;
    geometry_msgs::Point p_0;
    p_0.x = bottom_quad[0][0]; p_0.y = bottom_quad[0][1]; p_0.z = bottom_quad[0][2]; 
    geometry_msgs::Point p_1;
    p_1.x = bottom_quad[1][0]; p_1.y = bottom_quad[1][1]; p_1.z = bottom_quad[1][2];
    geometry_msgs::Point p_2;
    p_2.x = bottom_quad[2][0]; p_2.y = bottom_quad[2][1]; p_2.z = bottom_quad[2][2];
    geometry_msgs::Point p_3;
    p_3.x = bottom_quad[3][0]; p_3.y = bottom_quad[3][1]; p_3.z = bottom_quad[3][2];
    geometry_msgs::Point p_0_h = p_0; p_0_h.z += h; 
    geometry_msgs::Point p_1_h = p_1; p_1_h.z += h; 
    geometry_msgs::Point p_2_h = p_2; p_2_h.z += h; 
    geometry_msgs::Point p_3_h = p_3; p_3_h.z += h; 
    if (is_lidar(sensor_type)) {
      lidar_bbox_list_.points.push_back(p_0); lidar_bbox_list_.points.push_back(p_1);
      lidar_bbox_list_.points.push_back(p_1); lidar_bbox_list_.points.push_back(p_2);
      lidar_bbox_list_.points.push_back(p_2); lidar_bbox_list_.points.push_back(p_3);
      lidar_bbox_list_.points.push_back(p_3); lidar_bbox_list_.points.push_back(p_0);
      lidar_bbox_list_.points.push_back(p_0_h); lidar_bbox_list_.points.push_back(p_1_h);
      lidar_bbox_list_.points.push_back(p_1_h); lidar_bbox_list_.points.push_back(p_2_h);
      lidar_bbox_list_.points.push_back(p_2_h); lidar_bbox_list_.points.push_back(p_3_h);
      lidar_bbox_list_.points.push_back(p_3_h); lidar_bbox_list_.points.push_back(p_0_h);
      lidar_bbox_list_.points.push_back(p_0); lidar_bbox_list_.points.push_back(p_0_h);
      lidar_bbox_list_.points.push_back(p_1); lidar_bbox_list_.points.push_back(p_1_h);
      lidar_bbox_list_.points.push_back(p_2); lidar_bbox_list_.points.push_back(p_2_h);
      lidar_bbox_list_.points.push_back(p_3); lidar_bbox_list_.points.push_back(p_3_h);
    } else if (is_camera(sensor_type)) {
      camera_bbox_list_[cam_idx].points.push_back(p_0); camera_bbox_list_[cam_idx].points.push_back(p_1);
      camera_bbox_list_[cam_idx].points.push_back(p_1); camera_bbox_list_[cam_idx].points.push_back(p_2);
      camera_bbox_list_[cam_idx].points.push_back(p_2); camera_bbox_list_[cam_idx].points.push_back(p_3);
      camera_bbox_list_[cam_idx].points.push_back(p_3); camera_bbox_list_[cam_idx].points.push_back(p_0);
      camera_bbox_list_[cam_idx].points.push_back(p_0_h); camera_bbox_list_[cam_idx].points.push_back(p_1_h);
      camera_bbox_list_[cam_idx].points.push_back(p_1_h); camera_bbox_list_[cam_idx].points.push_back(p_2_h);
      camera_bbox_list_[cam_idx].points.push_back(p_2_h); camera_bbox_list_[cam_idx].points.push_back(p_3_h);
      camera_bbox_list_[cam_idx].points.push_back(p_3_h); camera_bbox_list_[cam_idx].points.push_back(p_0_h);
      camera_bbox_list_[cam_idx].points.push_back(p_0); camera_bbox_list_[cam_idx].points.push_back(p_0_h);
      camera_bbox_list_[cam_idx].points.push_back(p_1); camera_bbox_list_[cam_idx].points.push_back(p_1_h);
      camera_bbox_list_[cam_idx].points.push_back(p_2); camera_bbox_list_[cam_idx].points.push_back(p_2_h);
      camera_bbox_list_[cam_idx].points.push_back(p_3); camera_bbox_list_[cam_idx].points.push_back(p_3_h);
    } else if (is_radar(sensor_type)) {
      radar_bbox_list_.points.push_back(p_0); radar_bbox_list_.points.push_back(p_1);
      radar_bbox_list_.points.push_back(p_1); radar_bbox_list_.points.push_back(p_2);
      radar_bbox_list_.points.push_back(p_2); radar_bbox_list_.points.push_back(p_3);
      radar_bbox_list_.points.push_back(p_3); radar_bbox_list_.points.push_back(p_0);
      radar_bbox_list_.points.push_back(p_0_h); radar_bbox_list_.points.push_back(p_1_h);
      radar_bbox_list_.points.push_back(p_1_h); radar_bbox_list_.points.push_back(p_2_h);
      radar_bbox_list_.points.push_back(p_2_h); radar_bbox_list_.points.push_back(p_3_h);
      radar_bbox_list_.points.push_back(p_3_h); radar_bbox_list_.points.push_back(p_0_h);
      radar_bbox_list_.points.push_back(p_0); radar_bbox_list_.points.push_back(p_0_h);
      radar_bbox_list_.points.push_back(p_1); radar_bbox_list_.points.push_back(p_1_h);
      radar_bbox_list_.points.push_back(p_2); radar_bbox_list_.points.push_back(p_2_h);
      radar_bbox_list_.points.push_back(p_3); radar_bbox_list_.points.push_back(p_3_h);
    } else if (sensor_type == apollo_perception_standalone::SensorType::FUSION) {
      fusion_bbox_list_.points.push_back(p_0); fusion_bbox_list_.points.push_back(p_1);
      fusion_bbox_list_.points.push_back(p_1); fusion_bbox_list_.points.push_back(p_2);
      fusion_bbox_list_.points.push_back(p_2); fusion_bbox_list_.points.push_back(p_3);
      fusion_bbox_list_.points.push_back(p_3); fusion_bbox_list_.points.push_back(p_0);
      fusion_bbox_list_.points.push_back(p_0_h); fusion_bbox_list_.points.push_back(p_1_h);
      fusion_bbox_list_.points.push_back(p_1_h); fusion_bbox_list_.points.push_back(p_2_h);
      fusion_bbox_list_.points.push_back(p_2_h); fusion_bbox_list_.points.push_back(p_3_h);
      fusion_bbox_list_.points.push_back(p_3_h); fusion_bbox_list_.points.push_back(p_0_h);
      fusion_bbox_list_.points.push_back(p_0); fusion_bbox_list_.points.push_back(p_0_h);
      fusion_bbox_list_.points.push_back(p_1); fusion_bbox_list_.points.push_back(p_1_h);
      fusion_bbox_list_.points.push_back(p_2); fusion_bbox_list_.points.push_back(p_2_h);
      fusion_bbox_list_.points.push_back(p_3); fusion_bbox_list_.points.push_back(p_3_h);
    } else {
      AERROR << "objToBBox: unsupported sensor type: " << GetSensorType(sensor_type);
      return;
    }
    // add colors
    std_msgs::ColorRGBA points_color;
    if (is_camera(sensor_type)) {
      if (cam_idx == 0) {
        points_color.a = 1.0;
        points_color.r = 1.0;  // camera_long color objects are orange
        points_color.g = 0.5;
        points_color.b = 0.3;
      } else if (cam_idx == 1) {
        points_color.a = 1.0;
        points_color.r = 1.0;  // camera_short objects are yellow
        points_color.g = 1.0;
        points_color.b = 0.0;
      }
    } else if (is_radar(sensor_type)) {
      points_color.a = 1.0;  // radar objects are blue
      points_color.r = 0.0;
      points_color.g = 0.0;
      points_color.b = 1.0;
    } else if (sensor_type == apollo_perception_standalone::SensorType::FUSION) {
      if ((obj->fused_lidar && obj->fused_radar) || (obj->fused_lidar && obj->fused_cam)) {
        points_color.a = 1.0;  // matched fusion objects are lightcoral
        points_color.r = 0.9;
        points_color.g = 0.6;
        points_color.b = 0.6;
      } else {
        points_color.a = 1.0;  // unmatched fusion objects are firebrick
        points_color.r = 0.7;
        points_color.g = 0.2;
        points_color.b = 0.2;
      }
    } else {  // lidar objects 
      if (obj->type == apollo_perception_standalone::ObjectType::VEHICLE) {
        points_color.a = 1.0;
        points_color.r = 1.0;  // vehicle is red
        points_color.g = 0.0;
        points_color.b = 0.0;
      } else if (obj->type == apollo_perception_standalone::ObjectType::BICYCLE) {
        points_color.a = 1.0;
        points_color.r = 0.0;
        points_color.g = 1.0;  // bicycle is green
        points_color.b = 0.0;
      } else if (obj->type == apollo_perception_standalone::ObjectType::PEDESTRIAN) {
        points_color.a = 1.0;
        points_color.r = 0.0;
        points_color.g = 0.0;
        points_color.b = 1.0;  // pedestrian is blue
      } else {
        points_color.a = 1.0;
        points_color.r = 0.5;  // unknown is grey
        points_color.g = 0.5;
        points_color.b = 0.5;
      }
    }
    for (int i = 0; i < 24; i++) {
      if (is_lidar(sensor_type)) {
        lidar_bbox_list_.colors.push_back(points_color);
      } else if (is_camera(sensor_type)) {
        camera_bbox_list_[cam_idx].colors.push_back(points_color);
      } else if (is_radar(sensor_type)) {
        radar_bbox_list_.colors.push_back(points_color);
      } else if (sensor_type == apollo_perception_standalone::SensorType::FUSION) {
        fusion_bbox_list_.colors.push_back(points_color);
      } else {
        AERROR << "objToBBox: unsupported sensor type: " << GetSensorType(sensor_type);
      }
    }
  }
  return;
}

visualization_msgs::Marker PerceptionYX::objToMarker(std::shared_ptr<apollo_perception_standalone::Object> &obj,
                                                     std::string &frame_id,
                                                     ros::Time &timestamp,
                                                     std::string &ns,
                                                     int32_t id,
                                                     int32_t type) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = timestamp;
  marker.ns = ns;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = id;
  marker.color.r = 1.0; // yellow
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(0.1);
  marker.type = type;
  if (type == visualization_msgs::Marker::TEXT_VIEW_FACING) {
    marker.pose.position.x = obj->center(0);
    marker.pose.position.y = obj->center(1);
    marker.pose.position.z = obj->center(2) + 4.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 0.5;
    marker.text = "ID: " + std::to_string(obj->track_id) + "\n" 
                              + "v: " + to_string_with_precision(obj->velocity.norm(), 1);
  } else if (type == visualization_msgs::Marker::ARROW) {
    double marker_arrow_length = 1.0;
    marker.scale.x = 0.1; //shaft diameter
    marker.scale.y = 0.1; //head diameter
    marker.scale.z = 0.2; // head length
    geometry_msgs::Point marker_p1;
    marker_p1.x = obj->center(0);
    marker_p1.y = obj->center(1);
    marker_p1.z = obj->center(2);
    geometry_msgs::Point marker_p2;
    // marker_p2.x = obj->center(0) + obj->direction(0) * marker_arrow_length * obj->velocity.norm();
    // marker_p2.y = obj->center(1) + obj->direction(1) * marker_arrow_length * obj->velocity.norm();
    // marker_p2.z = obj->center(2) + obj->direction(2) * marker_arrow_length * obj->velocity.norm();
    marker_p2.x = obj->center(0) + obj->velocity(0);
    marker_p2.y = obj->center(1) + obj->velocity(1);
    marker_p2.z = obj->center(2) + obj->velocity(2);
    marker.points.push_back(marker_p1);
    marker.points.push_back(marker_p2);
  }
  return marker;
}

void PerceptionYX::processRadarData(const apollo_perception_standalone::ContiRadar &msg) {

  //   AWARN_IF(log_radar_) << "\033[1;32mprocess Radar data\033[0m, frame_id: " << msg->header.frame_id
  //     << " at time stamp: " << std::fixed << std::setprecision(12) << msg->header.stamp;

  //   if (!use_radar_) {
  //     AWARN_IF(log_radar_) << "radar deactivated!";
  //     return;
  //   }
    
  //   apollo_perception_standalone::RadarDetectorOptions options;
  //   std::shared_ptr<Eigen::Matrix4d> radar_trans = std::make_shared<Eigen::Matrix4d>(Eigen::Matrix4d::Identity());
  //   options.radar2world_pose = radar_trans.get();
  //   options.car_linear_speed = Eigen::Vector3f::Zero();
  //   std::vector<std::shared_ptr<apollo_perception_standalone::Object>> radar_objects;
  //   std::vector<apollo_perception_standalone::PolygonDType> map_polygons;

  //   // process radar data
  //   const clock_t time_start = clock();
  //   if (!radar_detector_->Detect(msg, map_polygons,
  //                                 options, &radar_objects)) {
  //     AERROR << "Radar perception error at timestamp: " << std::fixed
  //             << std::setprecision(12) << msg->header.stamp;
  //     return;
  //   }
  //   AWARN_IF(log_radar_) << "size of Objects from radar is: " << radar_objects.size();
  //   double radar_process_time = double(clock() - time_start) / CLOCKS_PER_SEC;
  //   AWARN_IF(log_radar_) << "\033[1;33m" << "Radar processing takes: " 
  //       << std::fixed << std::setprecision(6) << radar_process_time << "s at " 
  //       << 1.0 / radar_process_time << "Hz" << "\033[0m";
    
  //   // radar bbox line list definition
  //   radar_bbox_list_.header.stamp = msg->header.stamp;
  //   radar_bbox_list_.points.clear();
  //   radar_bbox_list_.colors.clear();
  //   radar_velocity_list_.markers.clear();

  //   // build bounding boxes / velocity markers
  //   int marker_count = 200;
  //   double marker_arrow_length = 1.0;
  //   for (auto obj : radar_objects) {
  //     // add bounding box
  //     objToBBox(obj, apollo_perception_standalone::SensorType::DELPHI_ESR);
  //     // add velocity marker
  //     // if (obj->velocity.norm() < 0.2) {  // skip close to static objects
  //     //   continue;
  //     // }
  //     // AWARN_IF(log_radar_) << "radar moving obj track id: " << obj->track_id;
  //     // AWARN_IF(log_radar_) << "radar moving obj velocity: " << obj->velocity;
  //     // velocity text
  //     ros::Time timestamp = msg->header.stamp;
  //     std::string ns = "radar velocity list";
  //     std::string frame_id = frame_id_fusion_;
  //     visualization_msgs::Marker velocity_text_marker = objToMarker(obj, 
  //                             frame_id, timestamp, ns, 
  //                             marker_count, visualization_msgs::Marker::TEXT_VIEW_FACING);
  //     radar_velocity_list_.markers.push_back(velocity_text_marker);
  //     marker_count++;
  //     // velocity arrow
  //     visualization_msgs::Marker velocity_arrow_marker = objToMarker(obj, 
  //                             frame_id, timestamp, ns, 
  //                             marker_count, visualization_msgs::Marker::ARROW);
  //     radar_velocity_list_.markers.push_back(velocity_arrow_marker);
  //     marker_count++;
  //   }
  //   // publish results
  //   radar_bbox_publisher_.publish(radar_bbox_list_);
  //   radar_velocity_publisher_.publish(radar_velocity_list_);

  //   // prepare for sensor fusion
  //   radar_objects_->objects = radar_objects;
  //   radar_objects_->sensor_type = apollo_perception_standalone::SensorType::DELPHI_ESR;
  //   radar_objects_->sensor_id = GetSensorType(radar_objects_->sensor_type);
  //   radar_objects_->timestamp = msg->header.stamp.toSec();
  //   radar_objects_->sensor2world_pose = Eigen::Matrix4d::Identity();

  //   // fusion
  //   // processFusion(radar_objects_);

   return;
}

bool PerceptionYX::projectCloudToImage( double timestamp, 
                            apollo_perception_standalone::pcl_util::VPointCloud &cloud_old,
                            const double intrinsic[], const double distortion[],
                            const double extrinsic[], int render_mode,
                            double min_v, double max_v,
                            const std::vector<cv::Scalar> &clr_vec,
                            int point_size, int thinkness, int point_type,
                            cv::Mat &out_image) {
  // get transformation from vehicle to lidar
  std::shared_ptr<Eigen::Matrix4d> trans = std::make_shared<Eigen::Matrix4d>();
  if (!getSensorTrans(timestamp, trans.get(), 
          FLAGS_lidar_tf2_child_frame_id, FLAGS_lidar_tf2_frame_id)) {
    AERROR << "failed to get trans at timestamp: " << timestamp;
    return false;
  }

  // new cloud in lidar coordinate
  apollo_perception_standalone::pcl_util::VPointCloud cloud;
  cloud.resize(cloud_old.size());
  for (size_t i = 0; i < cloud_old.points.size(); ++i) {
    const apollo_perception_standalone::pcl_util::VPoint& p = cloud_old.at(i);
    Eigen::Vector4d v(p.x, p.y, p.z, 1);
    v = (*trans) * v;
    apollo_perception_standalone::pcl_util::VPoint& tp = cloud.at(i);
    tp.x = v.x();
    tp.y = v.y();
    tp.z = v.z();
    tp.intensity = p.intensity;
  }

  //std::cout << cloud.points.size() << " " << out_image.size() << " " << clr_vec.size() << std::endl;
  if (cloud.empty() || out_image.empty() || clr_vec.empty()) {
    AERROR << "PROJECTION ERROR: cloud.empty() || out_image.empty() || clr_vec.empty()";
    return false;
  }
  point_size = point_size <= 0 ? 1 : point_size;
  int point_count = cloud.points.size();
  std::vector<cv::Point3f> pt3d_vec;
  std::vector<int> inten_vec;
  double rr[9];
  double tt[3];
  convert_rot_trans(extrinsic, rr, tt);
  double xx[3];
  double yy[3];
  float img_pt[2];
  int width = out_image.cols;
  int height = out_image.rows;
  float ratio = 0.5;
  float min_w = -ratio * width;
  float max_w = (1 + ratio) * width;
  float min_h = -ratio * height;
  float max_h = (1 + ratio) * height;
  for (int i = 0; i < point_count; ++i) {
    xx[0] = cloud.points[i].x;
    xx[1] = cloud.points[i].y;
    xx[2] = cloud.points[i].z;
    if (std::isnan(xx[0]) || std::isnan(xx[1]) || std::isnan(xx[2])) {
      continue;
    }
    yy[2] = rr[6] * xx[0] + rr[7] * xx[1] + rr[8] * xx[2] + tt[2];
    if (yy[2] < 1.0) {
      continue;
    }
    yy[2] = 1.0 / yy[2];
    yy[0] = rr[0] * xx[0] + rr[1] * xx[1] + rr[2] * xx[2] + tt[0];
    img_pt[0] = yy[0] * yy[2] * intrinsic[0] + intrinsic[2];
    if (img_pt[0] < min_w || img_pt[0] > max_w) {
      continue;
    }
    yy[1] = rr[3] * xx[0] + rr[4] * xx[1] + rr[5] * xx[2] + tt[1];
    img_pt[1] = yy[1] * yy[2] * intrinsic[1] + intrinsic[3];
    if (img_pt[1] < min_h || img_pt[1] > max_h) {
      continue;
    }
    pt3d_vec.push_back(cv::Point3f(xx[0], xx[1], xx[2]));
    inten_vec.push_back(cloud.points[i].intensity);
  }
  if (pt3d_vec.empty()) {
    AINFO << "PROJECTION ERROR: pt3d_vec.empty()";
    return false;
  }
  cv::Mat rvec;
  cv::Mat tvec;
  convert_cv_vec(extrinsic, rvec, tvec);
  cv::Mat camera_k;
  convert_camera_matrix(intrinsic, camera_k);
  cv::Mat camera_d;
  convert_camera_dist(distortion, camera_d);
  std::vector<cv::Point2f> img_pts;
  cv::projectPoints(pt3d_vec, rvec, tvec, camera_k, camera_d, img_pts);

  int clr_count = clr_vec.size();
  double kk = clr_count * 1.0 / (max_v - min_v);
  double bb = -min_v * clr_count / (max_v - min_v);
  for (int i = 0; i < img_pts.size(); ++i) {
    int idx = 0;
    if (render_mode == 1) {
      idx = int(kk * pt3d_vec[i].z + bb);
    } else if (render_mode == 2) {
      double distance = cv::norm(pt3d_vec[i]);
      idx = int(kk * distance + bb);
    } else if (render_mode == 3) {
      idx = int(kk * inten_vec[i] + bb);
    } else {
      idx = int(kk * inten_vec[i] + bb);
    }
    // std::cout << max_v << " " << min_v << " " << kk << " " << bb<< " " <<
    // pt3d_vec[i].z << " " << idx << std::endl;
    idx = idx < 0 ? 0 : idx;
    idx = idx >= clr_count ? clr_count - 1 : idx;
    cv::Scalar clr = clr_vec[idx];
    if (point_type == 1) {
      float x = img_pts[i].x;
      float y = img_pts[i].y;
      cv::line(out_image, cv::Point2f(x + point_size, y),
               cv::Point2f(x - point_size, y), clr, thinkness);
      cv::line(out_image, cv::Point2f(x, y + point_size),
               cv::Point2f(x, y - point_size), clr, thinkness);
    } else {
      cv::circle(out_image, img_pts[i], point_size, clr, thinkness);
    }
  }
  return true;
}

void PerceptionYX::processImageData(const sensor_msgs::ImageConstPtr& msg, const int cam_idx) {
  AWARN_IF(log_cam_) << "\033[1;32mprocess " << (cam_idx == 0 ? "Camera Long" : "Camera Short") 
    << " camera data\033[0m,"
    << " frame_id: " << msg->header.frame_id
    << " at timestamp: " << std::fixed << std::setprecision(12) << msg->header.stamp
    << " system time: " << ros::Time::now().toSec();

  // fix timestamp in apollo demo 2.0 bag
  ros::Time time_now = ros::Time::now();
    
  if (!use_cam_long_ && cam_idx == 0) {
    AWARN_IF(log_cam_) << "camera_long deactivated!";
    // return;
  }
  if (!use_cam_short_ && cam_idx == 1) {
    AWARN_IF(log_cam_) << "camera_back deactivated!";
    // return;
  }

  // camera bbox line list definition
  double timestamp = time_now.toSec();
  camera_bbox_list_[cam_idx].header.stamp = time_now;
  camera_bbox_list_[cam_idx].points.clear();
  camera_bbox_list_[cam_idx].colors.clear();
  
  cv::Mat image;
  if (!messageToMat(*msg, &image)) {
    AERROR << "Conversion MessageToMat failed!";
  } else {
    AINFO << "Conversion MessageToMat succeeded with type: " << image.type() 
          << " for camera id: " << cam_idx;
  }

  // process camera image
  const clock_t time_start = clock();
  if (cam_idx == 0 && use_cam_long_) {
    camera_long_process_->ImgCallback(time_now.toSec(), camera_long_objects_, image);
    AWARN_IF(log_cam_) << "size of SensorObjects from camera_long is: " << camera_long_objects_->objects.size();
    // plot objects on image
    objToImg(image, true,
                    camera_long_objects_->sensor2world_pose,
                    camera_long_objects_->sensor2world_pose_static, 
                    camera_long_objects_->objects,
                    (*(camera_long_objects_->camera_frame_supplement)),
                    cam_idx);
  } else if (cam_idx == 1 && use_cam_short_) {
    camera_short_process_->ImgCallback(time_now.toSec(), camera_short_objects_, image);
    AWARN_IF(log_cam_) << "size of SensorObjects from camera_short is: " << camera_short_objects_->objects.size();
    // plot objects on image
    objToImg(image, true,
                    camera_short_objects_->sensor2world_pose,
                    camera_short_objects_->sensor2world_pose_static, 
                    camera_short_objects_->objects,
                    (*(camera_short_objects_->camera_frame_supplement)),
                    cam_idx);
  }
  double camera_process_time = double(clock() - time_start) / CLOCKS_PER_SEC;
  AWARN_IF(log_cam_) << "\033[1;33m" << (cam_idx == 0 ? "Camera Long" : "Camera Short") 
      << " camera processing takes: " << std::fixed << std::setprecision(6) 
      << camera_process_time << "s at " << 1.0 / camera_process_time << "Hz" << "\033[0m";
  
  // create bounding boxes for objects
  if (cam_idx == 0 && use_cam_long_) {
    for (auto obj : camera_long_objects_->objects) {
      objToBBox(obj, apollo_perception_standalone::SensorType::CAMERA, cam_idx);
    }
  } else if (cam_idx == 1 && use_cam_short_) {
    for (auto obj : camera_short_objects_->objects) {
      objToBBox(obj, apollo_perception_standalone::SensorType::CAMERA, cam_idx);
    }
  }

  // project point clouds on image
  if (proj_ptcloud_ && cam_idx == 1) {
    //copy latest point cloud for projection
    apollo_perception_standalone::pcl_util::VPointCloud filtered_cloud_objects_copy;
    //VPointCloud::Ptr filtered_cloud_objects_copy_ptr(new VPointCloud);
    //filtered_cloud_objects_copy_ptr.reset (new VPointCloud (filtered_cloud_objects_copy));
    pthread_mutex_lock(&proj_lock_);
    //copyPointCloud(*filtered_cloud_objects_copy_ptr, filtered_cloud_objects_);
    filtered_cloud_objects_copy = *filtered_cloud_objects_ptr_;
    //VPointCloud::Ptr filtered_cloud_objects_copy_ptr = filtered_cloud_objects_.makeShared();
    pthread_mutex_unlock(&proj_lock_);
    if (!projectCloudToImage(
                timestamp, filtered_cloud_objects_copy, intrinsic_[cam_idx],
                distortion_[cam_idx], extrinsic_[cam_idx], render_mode_, min_v_, max_v_,
                clr_vec_, point_size_, thinkness_, point_type_, image)) {
      AWARN << "Error in Prjection" << std::endl;
    }
  }

  // publish results
  if (cam_idx == 0) {
    // image_long_ = *(cv_ptr->toImageMsg());
    if (!matToMessage(image, &image_long_)) {
      AERROR << "Conversion MatToMessage failed!";
    }
    camera_long_bbox_publisher_.publish(camera_bbox_list_[cam_idx]);
    image_long_publisher_.publish(image_long_);
  } else if (cam_idx == 1) {
    if (!matToMessage(image, &image_short_)) {
      AERROR << "Conversion MatToMessage failed!";
    }
    camera_short_bbox_publisher_.publish(camera_bbox_list_[cam_idx]);
    image_short_publisher_.publish(image_short_);
  }

   // prepare for sensor fusion
  if (cam_idx == 0 && use_cam_long_) {
    camera_long_objects_->sensor_type = apollo_perception_standalone::SensorType::CAMERA;
    camera_long_objects_->sensor_id = GetSensorType(camera_long_objects_->sensor_type);
    camera_long_objects_->timestamp = time_now.toSec();
    camera_long_objects_->sensor2world_pose = Eigen::Matrix4d::Identity();
    // fusion
    processFusion(camera_long_objects_);
  } else if (cam_idx == 1 && use_cam_short_) {
    camera_short_objects_->sensor_type = apollo_perception_standalone::SensorType::CAMERA;
    camera_short_objects_->sensor_id = GetSensorType(camera_short_objects_->sensor_type);
    camera_short_objects_->timestamp = time_now.toSec();
    camera_short_objects_->sensor2world_pose = Eigen::Matrix4d::Identity();
    // fusion
    processFusion(camera_short_objects_);
  }
}

/**
 * @brief process the point cloud and build bounding boxes
 * 
 * @param scan the point cloud pointer
 */
void PerceptionYX::processPointCloud(const apollo_perception_standalone::pcl_util::VPointCloudPtr &scan){
  // ----------------------------------------------------------------- //
  // ------------- Lidar Process --------------------------------------//
  // ----------------------------------------------------------------- //
  lidar_process_->Process(scan, filtered_cloud_ptr_, filtered_cloud_ground_ptr_);
  
  // sum point clouds of objects
  apollo_perception_standalone::pcl_util::PointCloudPtr filtered_cloud_objects_ptr(new apollo_perception_standalone::pcl_util::PointCloud);
  for (auto &object : lidar_process_->GetObjects()) {
    *filtered_cloud_objects_ptr += *(object->cloud);
  }

  // copy for point cloud projection
  //pthread_mutex_lock(&proj_lock_);
  filtered_cloud_objects_ptr_->points.resize(filtered_cloud_objects_ptr->size());
  for (size_t i = 0; i < filtered_cloud_objects_ptr->points.size(); i++) {
      filtered_cloud_objects_ptr_->points[i].x = filtered_cloud_objects_ptr->points[i].x;
      filtered_cloud_objects_ptr_->points[i].y = filtered_cloud_objects_ptr->points[i].y;
      filtered_cloud_objects_ptr_->points[i].z = filtered_cloud_objects_ptr->points[i].z;
      filtered_cloud_objects_ptr_->points[i].intensity = filtered_cloud_objects_ptr->points[i].intensity;
  }
  //pthread_mutex_unlock(&proj_lock_);

  // ----------------------------------------------------------------- //
  // ------------- Build Visualization Markers ------------------------//
  // ----------------------------------------------------------------- //
  // build bounding boxes / velocity markers
  int marker_count = 100;
  double marker_arrow_length = 1.0;
  for (auto obj : lidar_process_->GetObjects()) {
    // add bounding box
    objToBBox(obj, apollo_perception_standalone::SensorType::VELODYNE_64);
    // add velocity marker
    if (obj->velocity.norm() < 0.2) {  // skip close to static objects
      continue;
    }
    // AWARN_IF(log_lidar_) << "lidar moving obj track id: " << obj->track_id;
    // AWARN_IF(log_lidar_) << "lidar moving obj velocity: " << obj->velocity;
    // velocity text
    ros::Time timestamp = pcl_conversions::fromPCL(scan->header.stamp);
    std::string ns = "lidar velocity list";
    visualization_msgs::Marker velocity_text_marker = objToMarker(obj, 
                            frame_id_fusion_, timestamp, ns, 
                            marker_count, visualization_msgs::Marker::TEXT_VIEW_FACING);
    lidar_velocity_list_.markers.push_back(velocity_text_marker);
    marker_count++;
    // velocity arrow
    visualization_msgs::Marker velocity_arrow_marker = objToMarker(obj, 
                            frame_id_fusion_, timestamp, ns, 
                            marker_count, visualization_msgs::Marker::ARROW);
    lidar_velocity_list_.markers.push_back(velocity_arrow_marker);
    marker_count++;
  }
  return;
}

/**
 * @brief the point cloud subscriber callback function
 * 
 * @param scan the point cloud pointer
 */
void PerceptionYX::processLidarData(const apollo_perception_standalone::pcl_util::VPointCloudPtr &scan) {
  AWARN_IF(log_lidar_) << "\033[1;32mprocess Lidar data\033[0m,"
    << " frame_id: " << scan->header.frame_id
    << " timestamp: " << std::fixed << std::setprecision(12) << scan->header.stamp
    << " system time: " << ros::Time::now().toSec()
    << " pointcloud size: " << scan->points.size();

  // fix timestamp in apollo demo 2.0 bag
  scan->header.stamp = pcl_conversions::toPCL(ros::Time::now());

  if (!use_lidar_) {
    AWARN_IF(log_lidar_) << "lidar deactivated!";
    return;
  }

  // prepare point clouds
  filtered_cloud_ptr_.reset(new apollo_perception_standalone::pcl_util::VPointCloud);
  filtered_cloud_objects_ptr_.reset(new apollo_perception_standalone::pcl_util::VPointCloud);
  filtered_cloud_ground_ptr_.reset(new apollo_perception_standalone::pcl_util::VPointCloud);
  // update timestamp to local pc time
  // pcl_conversions::toPCL(ros::Time::now(), scan->header.stamp);
  // pass along original time stamp and frame ID
  filtered_cloud_ptr_->header.stamp = scan->header.stamp;
  filtered_cloud_ptr_->header.frame_id = frame_id_lidar_;
  filtered_cloud_ground_ptr_->header.stamp = scan->header.stamp;
  filtered_cloud_ground_ptr_->header.frame_id = frame_id_lidar_;
  filtered_cloud_objects_ptr_->header.stamp = scan->header.stamp;
  filtered_cloud_objects_ptr_->header.frame_id = frame_id_fusion_;  // objects in fusion frame
  
  // lidar bbox line list definition
  lidar_bbox_list_.header.stamp = pcl_conversions::fromPCL(scan->header.stamp);
  lidar_bbox_list_.points.clear();
  lidar_bbox_list_.colors.clear();
  // reset velocity markers
  lidar_velocity_list_.markers.clear();

  // process point cloud
  clock_t time_start = clock();
  processPointCloud(scan);
  AWARN_IF(log_lidar_) << "size of Objects from Lidar is: " 
      << lidar_process_->GetObjects().size();
  double lidar_process_time = double(clock() - time_start) / CLOCKS_PER_SEC;
  AWARN_IF(log_lidar_) << "\033[1;33m" << "Lidar processing takes: " 
      << std::fixed << std::setprecision(6) << lidar_process_time << "s at " 
      << 1.0 / lidar_process_time << "Hz" << "\033[0m";
  
  // publish point clouds
  filtered_cloud_publisher_.publish(filtered_cloud_ptr_);
  filtered_cloud_objects_publisher_.publish(filtered_cloud_objects_ptr_);
  filtered_cloud_ground_publisher_.publish(filtered_cloud_ground_ptr_);
  // publish bounding boxes / velocity
  lidar_bbox_publisher_.publish(lidar_bbox_list_);
  lidar_velocity_publisher_.publish(lidar_velocity_list_);

  // prepare for sensor fusion
  lidar_objects_->objects = lidar_process_->GetObjects();
  lidar_objects_->sensor_type = apollo_perception_standalone::SensorType::VELODYNE_64;
  lidar_objects_->sensor_id = GetSensorType(lidar_objects_->sensor_type);
  lidar_objects_->timestamp = pcl_conversions::fromPCL(scan->header.stamp).toSec();
  lidar_objects_->sensor2world_pose = Eigen::Matrix4d::Identity();

  // fusion
  processFusion(lidar_objects_);

  return;
}

void PerceptionYX::processFusion(
          std::shared_ptr<apollo_perception_standalone::SensorObjects> &in_sensor_objects) {
  AWARN_IF(log_fusion_) << "\033[1;32mprocess Fusion\033[0m,"
    << " at time stamp: " << std::fixed << std::setprecision(12) << in_sensor_objects->timestamp;
  
  // fused objects
  std::vector<std::shared_ptr<apollo_perception_standalone::Object>> fused_objects;

  // process fusion
  double time_start = clock();
  obstacle_fusion_.Process(in_sensor_objects, &fused_objects);

  // publish fusion results
  fusion_pub_mutex_.lock();
  publishFusion(fused_objects, in_sensor_objects->timestamp);
  fusion_pub_mutex_.unlock();

  // calculate time
  double fusion_process_time = double(clock() - time_start) / CLOCKS_PER_SEC;
  AWARN_IF(log_fusion_) << "\033[1;33m" << "Fusion processing takes: " 
      << std::fixed << std::setprecision(6) << fusion_process_time << "s at " 
      << 1.0 / fusion_process_time << "Hz" << "\033[0m";
  return;
}

void PerceptionYX::publishFusion(
          std::vector<std::shared_ptr<apollo_perception_standalone::Object>> &fused_objects, 
          double timestamp) {
  // fusion bbox line list definition
  ros::Time stamp = ros::Time(timestamp);
  fusion_bbox_list_.header.stamp = stamp;
  fusion_bbox_list_.points.clear();
  fusion_bbox_list_.colors.clear();
  fusion_velocity_list_.markers.clear();

  // build bounding boxes / velocity markers
  int marker_count = 300;
  double marker_arrow_length = 1.0;
  for (auto obj : fused_objects) {
    // add bounding box
    objToBBox(obj, apollo_perception_standalone::SensorType::FUSION);
    // add velocity marker
    // if (obj->velocity.norm() < 0.2) {  // skip close to static objects
    //   continue;
    // }
    // AWARN_IF(log_fusion_) << "fusion moving obj track id: " << obj->track_id;
    // AWARN_IF(log_fusion_) << "fusion moving obj velocity: " << obj->velocity;
    // velocity text
    std::string ns = "fusion velocity list";
    visualization_msgs::Marker velocity_text_marker = objToMarker(obj, 
                            frame_id_fusion_, stamp, ns, 
                            marker_count, visualization_msgs::Marker::TEXT_VIEW_FACING);
    fusion_velocity_list_.markers.push_back(velocity_text_marker);
    marker_count++;
    // velocity arrow
    visualization_msgs::Marker velocity_arrow_marker = objToMarker(obj, 
                            frame_id_fusion_, stamp, ns, 
                            marker_count, visualization_msgs::Marker::ARROW);
    fusion_velocity_list_.markers.push_back(velocity_arrow_marker);
    marker_count++;
  }

  // publish bounding boxes / velocity
  fusion_bbox_publisher_.publish(fusion_bbox_list_);
  fusion_velocity_publisher_.publish(fusion_velocity_list_);

  return;
}

bool PerceptionYX::messageToMat(const sensor_msgs::Image &msg,
                                        cv::Mat *img) {
  *img = cv::Mat(msg.height, msg.width, CV_8UC3);
  int pixel_num = msg.width * msg.height;
  if (msg.encoding.compare("yuyv") == 0) {
    // AERROR << "yuyv format not supported";
    // return false;
    unsigned char *yuv = (unsigned char *)&(msg.data[0]);
    PerceptionYX::Yuyv2rgb(yuv, img->data, pixel_num);
    cv::cvtColor(*img, *img, CV_RGB2BGR);
  } else {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
        // cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    *img = cv_ptr->image;
  }
  return true;
}

bool PerceptionYX::matToMessage(const cv::Mat &img,
                                        sensor_msgs::Image *msg) {
  if (img.type() == CV_8UC1) {
    sensor_msgs::fillImage(*msg, sensor_msgs::image_encodings::MONO8, img.rows,
                           img.cols, static_cast<unsigned int>(img.step),
                           img.data);
    return true;
  } else if (img.type() == CV_8UC3) {
    // https://answers.ros.org/question/11312/what-is-image-step/
    sensor_msgs::fillImage(*msg, sensor_msgs::image_encodings::BGR8, img.rows,
                           img.cols, static_cast<unsigned int>(img.step),
                           img.data);
    return true;
  } else if (img.type() == CV_32FC1) {
    cv::Mat uc_img(img.rows, img.cols, CV_8UC1);
    uc_img.setTo(cv::Scalar(0));
    for (int h = 0; h < uc_img.rows; ++h) {
      for (int w = 0; w < uc_img.cols; ++w) {
        // if (img.at<float>(h, w) >= ln_msk_threshold_) {
        if (img.at<float>(h, w) >= 0.5) {
          uc_img.at<unsigned char>(h, w) = 1;
        }
      }
    }

    sensor_msgs::fillImage(*msg, sensor_msgs::image_encodings::MONO8,
                           uc_img.rows, uc_img.cols,
                           static_cast<unsigned int>(uc_img.step), uc_img.data);
    return true;
  } else {
    AERROR << "invalid input Mat type: " << img.type();
    return false;
  }
}

bool PerceptionYX::getSensorTrans(const double query_time, Eigen::Matrix4d* trans,
  std::string parent_frame_id, std::string child_frame_id) {
  if (!trans) {
    AERROR << "failed to get trans, the trans ptr can not be nullptr";
    return false;
  }

  ros::Time query_stamp(query_time);
  // const double kTf2BuffSize = FLAGS_tf2_buff_in_ms / 1000.0;
  std::string err_msg;
  if (!tf2_listener_.canTransform(parent_frame_id,
                                child_frame_id,
                                query_stamp,
                              //  ros::Time(0),
                                &err_msg)) {
    AERROR << "Cannot transform frame: " << parent_frame_id
            << " to frame " << child_frame_id
            << " , err: " << err_msg
            << ". Frames: " << tf2_listener_.allFramesAsString();
    return false;
  }

  tf::StampedTransform stamped_transform;
  // geometry_msgs::TransformStamped transform_stamped;
  try {
    tf2_listener_.lookupTransform(parent_frame_id, 
      child_frame_id, query_stamp, stamped_transform);
  } catch (tf2::TransformException& ex) {
    AERROR << "Exception: " << ex.what();
    return false;
  }
  Eigen::Affine3d affine_3d;
  tf::transformTFToEigen(stamped_transform, affine_3d);
  *trans = affine_3d.matrix();

  // AWARN << "get " << parent_frame_id << " to "
  //         << child_frame_id << " trans: \n" << *trans;
  return true;
}

} // namespace perception_yx
