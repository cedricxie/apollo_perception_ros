cmake_minimum_required(VERSION 2.8.8)
project(apollo_perception_ros)

add_compile_options(-std=c++11)
#set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set (CUDA_PROPAGATE_HOST_FLAGS OFF)

#https://github.com/PointCloudLibrary/pcl/blob/master/cmake/pcl_verbosity.cmake
#set(PCL_VERBOSITY_LEVEL L_ALWAYS)

# Pass options to NVCC
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}; -std=c++11; -D__CORRECT_ISO_CPP11_MATH_H_PROTO; -arch=sm_30")
#message("CUDA_NVCC_FLAGS: " ${CUDA_NVCC_FLAGS})

############### catkin dependencies ######################
set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    roslaunch 
    rostest
    pcl_ros
    sensor_msgs
    tf
    tf_conversions
    cv_bridge)
find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_DEPS}
             pcl_conversions
             angles
             )

############### cuda 8.0 ######################
find_package(CUDA REQUIRED)
#set(CUDA_INCLUDE_DIR "/usr/local/cuda/include/")
#set(CUDA_LIB_FOLDER_DIR "/usr/local/cuda/lib64")
#link_directories(${CUDA_LIB_FOLDER_DIR})
# https://cmake.org/cmake/help/v3.0/module/FindCUDA.html
#message("CUDA Found: " ${CUDA_FOUND})
#message("CUDA Include path: " ${CUDA_INCLUDE_DIRS})
set( CUDA_ALL_LIBS ${CUDA_LIBRARIES} 
                   #${CUDA_CUFFT_LIBRARIES} 
                   #${CUDA_CUBLAS_LIBRARIES} 
                   #${CUDA_cupti_LIBRARY} 
                   #${CUDA_curand_LIBRARY} 
                   #${CUDA_cusparse_LIBRARY} 
                   ${CUDA_npp_LIBRARY} 
                   #${CUDA_nppc_LIBRARY} 
                   #${CUDA_nppi_LIBRARY} 
                   #${CUDA_npps_LIBRARY} 
                   #${CUDA_nvcuvenc_LIBRARY} 
                   #${CUDA_nvcuvid_LIBRARY}
                   )
#message("CUDA All libraries: " ${CUDA_ALL_LIBS})

############### boost 1.54.0 ######################
find_package(Boost REQUIRED)
#message("Boost Include path: " ${Boost_INCLUDE_DIR})
#message("Boost library path: " ${Boost_LIBRARIES})

############### pcl 1.7.2 ######################
find_package(PCL REQUIRED QUIET)
#message("PCL Include path: " ${PCL_INCLUDE_DIRS})
#message("PCL libraries: " ${PCL_LIBRARIES})

############### protobuf 3.3 ######################
include(FindProtobuf)
find_package(Protobuf 3.3 REQUIRED EXACT)
#message("Protobuf Include path: " ${PROTOBUF_INCLUDE_DIR})
#message("Protobuf libraries: " ${PROTOBUF_LIBRARIES})
#message("Protobuf library: " ${PROTOBUF_LIBRARY})

############### opencv 2.4.8 ######################
find_package( OpenCV REQUIRED COMPONENTS core highgui imgproc)
#message("OpenCV Include path: " ${OpenCV_INCLUDE_DIRS})
#message("OpenCV Libraries: " ${OpenCV_LIBS})

############### glog 0.3.3 ######################
set(GLOG_INCLUDE_DIR "/usr/local/include/")
set(GLOG_LIBRARY "/usr/local/lib/libglog.so")

############### gflags 2.0 ######################
set(GFLAGS_INCLUDE_DIR "/usr/local/include/")
set(GFLAGS_LIBRARY "/usr/local/lib/libgflags.so")

############### caffe from apollo ######################
set(CAFFE_INCLUDE_DIR "/usr/include/")
set(CAFFE_LIBRARY "/usr/lib/x86_64-linux-gnu/libcaffe.so")
set(CAFFE_PROTO_LIBRARY "/usr/lib/x86_64-linux-gnu/libproto.a")
add_library(caffe_lib SHARED IMPORTED)
set_target_properties(caffe_lib PROPERTIES IMPORTED_LOCATION ${CAFFE_LIBRARY})
set( CAFFE_ALL_LIBS caffe_lib
                    ${CAFFE_PROTO_LIBRARY})

############### include directories ######################
include_directories(include
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIR}
                    ${PROTOBUF_INCLUDE_DIR}
                    ${CAFFE_INCLUDE_DIR}
                    ${CUDA_INCLUDE_DIR}
                    ${GLOG_INCLUDE_DIR}
                    ${GFLAGS_INCLUDE_DIR}
                    ${OpenCV_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})

############### define third party libs ######################
set( THIRD_PARTY_LIBS
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PCL_LIBRARIES}
        ${PROTOBUF_LIBRARY}
        ${GLOG_LIBRARY}
        ${GFLAGS_LIBRARY}
        ${OpenCV_LIBS}
        yaml-cpp)

############### subfolders ######################
add_subdirectory( src )