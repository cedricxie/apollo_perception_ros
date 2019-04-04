############### roi_filter_lib ######################
add_library( roi_filter_proto_lib STATIC ${PROJECT_SOURCE_DIR}/include/lidar/proto/roi_filter_config.pb.cc)

add_library( roi_filter_lib STATIC ${PROJECT_SOURCE_DIR}/src/lidar/roi_filter/roi_filter.cc)

target_link_libraries(roi_filter_lib roi_filter_proto_lib
                                     ${PCL_LIBRARIES})

############### cnnseg_lib ######################
add_library( cnnseg_proto_lib STATIC ${PROJECT_SOURCE_DIR}/include/lidar/proto/cnnseg.pb.cc 
                                     ${PROJECT_SOURCE_DIR}/include/lidar/proto/cnn_segmentation_config.pb.cc)

add_library( cnnseg_lib STATIC ${PROJECT_SOURCE_DIR}/src/lidar/segmentation/cnnseg/cnn_segmentation.cpp 
                               ${PROJECT_SOURCE_DIR}/src/lidar/segmentation/cnnseg/feature_generator.cpp)

target_link_libraries(cnnseg_lib cnnseg_proto_lib
                                 ${CAFFE_ALL_LIBS})

############### object_filter_lib ######################
add_library( object_filter_proto_lib STATIC ${PROJECT_SOURCE_DIR}/include/lidar/proto/low_object_filter_config.pb.cc)

add_library( object_filter_lib STATIC ${PROJECT_SOURCE_DIR}/src/lidar/object_filter/low_object_filter.cc)

target_link_libraries(object_filter_lib object_filter_proto_lib)

############### object_builder_lib ######################
add_library( object_builder_lib STATIC ${PROJECT_SOURCE_DIR}/src/lidar/object_builder/min_box.cc)

target_link_libraries(object_builder_lib ${PCL_LIBRARIES} )

############### tracker_lib ######################
add_library( tracker_proto_lib STATIC ${PROJECT_SOURCE_DIR}/include/lidar/proto/tracker_config.pb.cc)

add_library( tracker_lib STATIC ${PROJECT_SOURCE_DIR}/src/lidar/tracker/feature_descriptor.cc
                                ${PROJECT_SOURCE_DIR}/src/lidar/tracker/hm_tracker.cc
                                ${PROJECT_SOURCE_DIR}/src/lidar/tracker/hungarian_matcher.cc
                                ${PROJECT_SOURCE_DIR}/src/lidar/tracker/kalman_filter.cc
                                ${PROJECT_SOURCE_DIR}/src/lidar/tracker/object_track.cc
                                ${PROJECT_SOURCE_DIR}/src/lidar/tracker/track_object_distance.cc
                                ${PROJECT_SOURCE_DIR}/src/lidar/tracker/tracked_object.cc)

target_link_libraries(tracker_lib common_lib
                                  util_lib
                                  tracker_proto_lib)