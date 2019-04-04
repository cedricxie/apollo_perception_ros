############### camera_proto_lib ######################
add_library( camera_proto_lib STATIC ${PROJECT_SOURCE_DIR}/include/camera/proto/geometry_camera_converter_config.pb.cc
                             ${PROJECT_SOURCE_DIR}/include/camera/proto/tracking_feature.pb.cc
                             ${PROJECT_SOURCE_DIR}/include/camera/proto/yolo_camera_detector_config.pb.cc
                             ${PROJECT_SOURCE_DIR}/include/camera/proto/yolo.pb.cc)
#target_link_libraries( camera_proto_lib util_lib) 

############### camera_common_lib ######################
add_library( camera_common_lib STATIC ${PROJECT_SOURCE_DIR}/src/camera/common/cnn_adapter.cc
                             ${PROJECT_SOURCE_DIR}/src/camera/common/projector.cc
                             ${PROJECT_SOURCE_DIR}/src/camera/common/util.cc)
#target_link_libraries( camera_common_lib util_lib)                             

############### camera_converter_lib ######################
add_library( camera_converter_lib STATIC ${PROJECT_SOURCE_DIR}/src/camera/converter/geometry_camera_converter.cc)
target_link_libraries( camera_converter_lib util_lib
                                            common_lib)     

############### camera_detector_lib ######################
add_library( camera_detector_lib STATIC ${PROJECT_SOURCE_DIR}/src/camera/detector/common/feature_extractor.cc
                                    ${PROJECT_SOURCE_DIR}/src/camera/detector/yolo_camera_detector/util.cc
                                    ${PROJECT_SOURCE_DIR}/src/camera/detector/yolo_camera_detector/yolo_camera_detector.cc)
target_link_libraries( camera_detector_lib util_lib
                                           common_lib
                                           cuda_util_lib
                                           camera_proto_lib
                                           camera_common_lib
                                           ${CAFFE_ALL_LIBS}
                                           ${CUDA_LIBRARIES}) 

############### camera_filter_lib ######################
add_library( camera_filter_lib STATIC ${PROJECT_SOURCE_DIR}/src/camera/filter/object_camera_extended_kalman_filter.cc
                                    ${PROJECT_SOURCE_DIR}/src/camera/filter/object_camera_filter.cc)
#target_link_libraries( camera_filter_lib util_lib) 

############### camera_motion_lib ######################
add_library( camera_motion_lib STATIC ${PROJECT_SOURCE_DIR}/src/camera/motion/plane_motion.cc)
#target_link_libraries( camera_motion_lib util_lib) 

############### camera_tracker_lib ######################
add_library( camera_tracker_lib STATIC ${PROJECT_SOURCE_DIR}/src/camera/tracker/cascaded_camera_tracker.cc
                                    ${PROJECT_SOURCE_DIR}/src/camera/tracker/cascaded_camera_tracker_util.cc
                                    ${PROJECT_SOURCE_DIR}/src/camera/tracker/cs2d/cs2d_affinity_tracker.cc
                                    ${PROJECT_SOURCE_DIR}/src/camera/tracker/dlf/dlf_affinity_tracker.cc
                                    ${PROJECT_SOURCE_DIR}/src/camera/tracker/kcf/kcf_affinity_tracker.cc
                                    ${PROJECT_SOURCE_DIR}/src/camera/tracker/kcf/kcf_components.cc)
#target_link_libraries( camera_tracker_lib util_lib) 

############### camera_transformer_lib ######################
add_library( camera_transformer_lib STATIC ${PROJECT_SOURCE_DIR}/src/camera/transformer/flat_camera_transformer.cc)
#target_link_libraries( camera_transformer_lib util_lib) 
