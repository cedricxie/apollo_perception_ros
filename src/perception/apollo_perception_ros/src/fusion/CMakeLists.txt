############### fusion_proto_lib ######################
add_library( fusion_proto_lib STATIC ${PROJECT_SOURCE_DIR}/include/fusion/proto/async_fusion_config.pb.cc
                                     ${PROJECT_SOURCE_DIR}/include/fusion/proto/probabilistic_fusion_config.pb.cc)

############### fusion_probabilistic_lib ######################
add_library( fusion_probabilistic_lib STATIC ${PROJECT_SOURCE_DIR}/src/fusion/probabilistic_fusion/dst_evidence.cc
                                             ${PROJECT_SOURCE_DIR}/src/fusion/probabilistic_fusion/pbf_base_track_object_matcher.cc
                                             ${PROJECT_SOURCE_DIR}/src/fusion/probabilistic_fusion/pbf_hm_track_object_matcher.cc
                                             ${PROJECT_SOURCE_DIR}/src/fusion/probabilistic_fusion/pbf_imf_fusion.cc
                                             ${PROJECT_SOURCE_DIR}/src/fusion/probabilistic_fusion/pbf_kalman_motion_fusion.cc
                                             ${PROJECT_SOURCE_DIR}/src/fusion/probabilistic_fusion/pbf_sensor_manager.cc
                                             ${PROJECT_SOURCE_DIR}/src/fusion/probabilistic_fusion/pbf_sensor_monitor.cc
                                             ${PROJECT_SOURCE_DIR}/src/fusion/probabilistic_fusion/pbf_sensor_object.cc
                                             ${PROJECT_SOURCE_DIR}/src/fusion/probabilistic_fusion/pbf_sensor.cc
                                             ${PROJECT_SOURCE_DIR}/src/fusion/probabilistic_fusion/pbf_track_manager.cc
                                             ${PROJECT_SOURCE_DIR}/src/fusion/probabilistic_fusion/pbf_track_object_distance.cc
                                             ${PROJECT_SOURCE_DIR}/src/fusion/probabilistic_fusion/pbf_track.cc
                                             ${PROJECT_SOURCE_DIR}/src/fusion/probabilistic_fusion/probabilistic_fusion.cc)

target_link_libraries(fusion_probabilistic_lib fusion_proto_lib
                                               util_lib
                                               common_lib
                                               #${GFLAGS_LIBRARY}
                                               )

############### fusion_async_lib ######################
add_library( fusion_async_lib STATIC ${PROJECT_SOURCE_DIR}/src/fusion/async_fusion/async_fusion.cc)

target_link_libraries(fusion_async_lib fusion_proto_lib
                                       fusion_probabilistic_lib)