############### proto_lib ######################
add_library( proto_lib STATIC ${PROJECT_SOURCE_DIR}/include/proto/error_code.pb.cc 
                              ${PROJECT_SOURCE_DIR}/include/proto/header.pb.cc
                              ${PROJECT_SOURCE_DIR}/include/proto/can_card_parameter.pb.cc
                              ${PROJECT_SOURCE_DIR}/include/proto/geometry.pb.cc
                              ${PROJECT_SOURCE_DIR}/include/proto/sensor_canbus_conf.pb.cc
                              ${PROJECT_SOURCE_DIR}/include/proto/perception_obstacle.pb.cc)

############### util_lib ######################
add_library( util_lib STATIC ${PROJECT_SOURCE_DIR}/src/util/aabox2d.cc 
                             ${PROJECT_SOURCE_DIR}/src/util/box2d.cc
                             ${PROJECT_SOURCE_DIR}/src/util/file.cc
                             ${PROJECT_SOURCE_DIR}/src/util/geometry_util.cc
                             ${PROJECT_SOURCE_DIR}/src/util/graph_util.cc
                             ${PROJECT_SOURCE_DIR}/src/util/hungarian_bigraph_matcher.cc
                             ${PROJECT_SOURCE_DIR}/src/util/line_segment2d.cc
                             ${PROJECT_SOURCE_DIR}/src/util/math_utils.cc
                             ${PROJECT_SOURCE_DIR}/src/util/matrix_operations.cc
                             ${PROJECT_SOURCE_DIR}/src/util/polygon2d.cc
                             ${PROJECT_SOURCE_DIR}/src/util/pose_util.cc
                             ${PROJECT_SOURCE_DIR}/src/util/string_util.cc
                             ${PROJECT_SOURCE_DIR}/src/util/timer.cc
                             ${PROJECT_SOURCE_DIR}/src/util/util.cc
                             ${PROJECT_SOURCE_DIR}/src/util/vec2d.cc)
target_link_libraries( util_lib ${Boost_LIBRARIES}
                                ${PROTOBUF_LIBRARY}
                                ${GLOG_LIBRARY})                             

############### common_lib ######################
add_library( common_lib STATIC ${PROJECT_SOURCE_DIR}/src/common/object.cc 
                               ${PROJECT_SOURCE_DIR}/src/common/object_supplement.cc
                               ${PROJECT_SOURCE_DIR}/src/common/types.cc
                               ${PROJECT_SOURCE_DIR}/src/common/config_gflags.cc
                               ${PROJECT_SOURCE_DIR}/src/common/perception_gflags.cc
                               )
target_link_libraries( common_lib util_lib 
                                  proto_lib
                                  ${GFLAGS_LIBRARY}
                                  ${Boost_LIBRARIES}
                                  ${PCL_LIBRARIES}   
                                  ${OpenCV_LIBS})

############### cuda_util_lib ######################
cuda_add_library( cuda_util_lib STATIC ${PROJECT_SOURCE_DIR}/src/cuda_util/block_uf.cu
                              ${PROJECT_SOURCE_DIR}/src/cuda_util/connected_component_gpu.cu
                              ${PROJECT_SOURCE_DIR}/src/cuda_util/network.cu
                              ${PROJECT_SOURCE_DIR}/src/cuda_util/region_output.cc
                              ${PROJECT_SOURCE_DIR}/src/cuda_util/region_output.cu
                              ${PROJECT_SOURCE_DIR}/src/cuda_util/undistortion.cu
                              ${PROJECT_SOURCE_DIR}/src/cuda_util/util.cu
                               )
target_link_libraries( cuda_util_lib util_lib 
                                     ${CUDA_ALL_LIBS} )

############### config_manager_lib ######################
add_library( config_manager_lib STATIC ${PROJECT_SOURCE_DIR}/src/config_manager/calibration_config_manager.cc)
target_link_libraries( config_manager_lib util_lib 
                                          cuda_util_lib)

############### subfolders ######################
add_subdirectory( camera )
add_subdirectory( lidar )
add_subdirectory( radar )
add_subdirectory( fusion )

############### lidar_process_lib ######################
add_library( lidar_process_lib STATIC ${PROJECT_SOURCE_DIR}/src/lidar_process.cpp)

target_link_libraries(lidar_process_lib roi_filter_lib
                                        cnnseg_proto_lib
                                        cnnseg_lib
                                        object_filter_proto_lib
                                        object_filter_lib
                                        object_builder_lib
                                        tracker_lib
                                        ${THIRD_PARTY_LIBS})

############### camera_process_lib ######################
add_library( camera_process_lib STATIC ${PROJECT_SOURCE_DIR}/src/camera_process.cc)

target_link_libraries(camera_process_lib camera_proto_lib
                                         camera_common_lib
                                         camera_converter_lib
                                         camera_detector_lib
                                         camera_filter_lib
                                         camera_motion_lib
                                         camera_tracker_lib
                                         camera_transformer_lib
                                         util_lib
                                         common_lib
                                         config_manager_lib
                                         ${THIRD_PARTY_LIBS})

############### obstacle_fusion_lib ######################
add_library( obstacle_fusion_lib STATIC ${PROJECT_SOURCE_DIR}/src/obstacle_fusion.cc)

target_link_libraries(obstacle_fusion_lib fusion_probabilistic_lib
                                          fusion_async_lib)
                                              
############### executable ######################
add_executable(perception_yx_node perception_yx_node.cpp perception_yx.cpp)

target_link_libraries(perception_yx_node lidar_process_lib
                                         camera_process_lib
                                         radar_modest_lib 
                                         obstacle_fusion_lib
                                         ${CUDA_LIBRARIES})

# install the files needed to run this package
install(TARGETS perception_yx_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)