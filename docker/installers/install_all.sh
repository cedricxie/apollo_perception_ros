#!/usr/bin/env bash

# Fail on first error.
set -e

echo "Preparing package installation..."
apt-get update -y && \
    apt-get install -y \
    apt-transport-https \
    build-essential \
    software-properties-common \
    bc \
    cmake \
    make \
    g++ \
    cppcheck \
    unzip \
    zip \
    wget \
    locate \
    git \
    nano \
    libboost-all-dev \
    libopencv-dev \
    libeigen3-dev \
    libpcap-dev \
    libyaml-cpp-dev \
    autoconf \
    automake \
    libtool \
    curl \
    doxygen-gui

echo "Installing caffe..."
apt-get update -y && apt-get install -y \
    libflann-dev \
    libopenblas-dev \
    libqhull-dev 

wget https://apollocache.blob.core.windows.net/apollo-docker/caffe_x86.tar.gz
tar xzf caffe_x86.tar.gz
mv caffe_x86/output-GPU/include/caffe /usr/include/
mv caffe_x86/output-GPU/lib/* /usr/lib/x86_64-linux-gnu/
# Clean up.
rm -fr caffe_x86.tar.gz caffe_x86

echo "Installing pcl..."
apt-get update -y && \
    apt-get install -y \
    libvtk-java \
    python-vtk \
    tcl-vtk \
    libvtk5-dev \
    libvtk5-qt4-dev \
    libusb-1.0-0-dev

wget https://apollocache.blob.core.windows.net/apollo-docker/pcl-1.7_x86.tar.gz
tar xzf pcl-1.7_x86.tar.gz
mkdir -p /usr/local/include/pcl-1.7
mv pcl-1.7_x86/include/pcl /usr/local/include/pcl-1.7/
mv pcl-1.7_x86/lib/* /usr/local/lib/
mv pcl-1.7_x86/share/pcl-1.7 /usr/local/share/
# Clean up.
rm -fr pcl-1.7_x86.tar.gz pcl-1.7_x86

echo "Installing protobuf..."
wget https://github.com/google/protobuf/releases/download/v3.3.0/protobuf-cpp-3.3.0.tar.gz
tar xzf protobuf-cpp-3.3.0.tar.gz

pushd protobuf-3.3.0
./configure
make -j4
#make check
make install
ldconfig
popd
# Clean up.
rm -fr protobuf-cpp-3.3.0.tar.gz protobuf-3.3.0

# Fix protobuf headers
# wget https://github.com/protocolbuffers/protobuf/blob/v3.3.0/src/google/protobuf/stubs/stringprintf.h
# wget https://github.com/protocolbuffers/protobuf/blob/v3.3.0/src/google/protobuf/stubs/strutil.h
pushd /tmp/installers/headers
mv stringprintf.h /usr/local/include/google/protobuf/stubs/
mv strutil.h /usr/local/include/google/protobuf/stubs/
popd

echo "Installing gflags and glog..."
wget https://github.com/gflags/gflags/archive/v2.0.tar.gz
tar xzf v2.0.tar.gz
pushd gflags-2.0
./configure
make -j4
make install
popd
# Install glog which also depends on gflags.
wget https://github.com/google/glog/archive/v0.3.3.tar.gz
tar xzf v0.3.3.tar.gz
pushd glog-0.3.3
./configure
make -j4
make install
popd
# Clean up.
rm -fr v2.0.tar.gz gflags-2.0 v0.3.3.tar.gz glog-0.3.3

echo "Installing ros indigo..."
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
apt-get update -y && \
    apt-get install -y \
    ros-indigo-desktop
rosdep init

# install catkin build tools
apt-get install -y python-catkin-tools