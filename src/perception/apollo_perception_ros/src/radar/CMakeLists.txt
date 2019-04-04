############### radar_modest_lib ######################
add_library( radar_modest_proto_lib STATIC ${PROJECT_SOURCE_DIR}/include/radar/proto/conti_radar_conf.pb.cc
                                           ${PROJECT_SOURCE_DIR}/include/radar/proto/conti_radar.pb.cc
                                           ${PROJECT_SOURCE_DIR}/include/radar/proto/delphi_esr.pb.cc
                                           ${PROJECT_SOURCE_DIR}/include/radar/proto/radar.pb.cc
                                           ${PROJECT_SOURCE_DIR}/include/radar/proto/modest_radar_detector_config.pb.cc)

target_link_libraries(radar_modest_proto_lib proto_lib)

add_library( radar_modest_lib STATIC ${PROJECT_SOURCE_DIR}/src/radar/modest/conti_radar_id_expansion.cc
                                     ${PROJECT_SOURCE_DIR}/src/radar/modest/conti_radar_util.cc
                                     ${PROJECT_SOURCE_DIR}/src/radar/modest/modest_radar_detector.cc
                                     ${PROJECT_SOURCE_DIR}/src/radar/modest/object_builder.cc
                                     ${PROJECT_SOURCE_DIR}/src/radar/modest/radar_track_manager.cc
                                     ${PROJECT_SOURCE_DIR}/src/radar/modest/radar_track.cc
                                     ${PROJECT_SOURCE_DIR}/src/radar/modest/radar_util.cc)

target_link_libraries(radar_modest_lib radar_modest_proto_lib)