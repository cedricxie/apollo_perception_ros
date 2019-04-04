#!/usr/bin/env bash

# Fail on first error.
set -e

echo "Preparing package installation..."

sudo apt-get update -y && \
    sudo apt-get install -y \
    apt-transport-https \
    build-essential \
    software-properties-common \
    bc \
    cmake \
    cppcheck \
    unzip \
    wget \
    locate \
    sudo \
    curl \
    debconf-utils \
    doxygen \
    git \
    nano \
    google-perftools \
    libeigen3-dev \
    libatlas-base-dev \
    libatlas-dev \
    libblas-dev \
    libboost-all-dev \
    libopencv-dev \
    libcurl4-openssl-dev \
    libfreetype6-dev \
    liblapack-dev \
    libpcap-dev \
    libhdf5-dev \
    libleveldb-dev \
    liblmdb-dev \
    libsnappy-dev \
    python-dev \
    python-pip \
    python-numpy \
    gfortran \
    realpath \
    zip && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install NCCL for multi-GPU communication
wget https://github.com/NVIDIA/nccl/releases/download/v1.2.3-1%2Bcuda8.0/libnccl1_1.2.3-1.cuda8.0_amd64.deb && \
  dpkg -i libnccl1_1.2.3-1.cuda8.0_amd64.deb && \
  rm libnccl1_1.2.3-1.cuda8.0_amd64.deb && \
wget https://github.com/NVIDIA/nccl/releases/download/v1.2.3-1%2Bcuda8.0/libnccl-dev_1.2.3-1.cuda8.0_amd64.deb && \
  dpkg -i libnccl-dev_1.2.3-1.cuda8.0_amd64.deb && \
  rm libnccl-dev_1.2.3-1.cuda8.0_amd64.deb

# Clone Caffe repo and move into it
cd /root && git clone https://github.com/BVLC/caffe.git && cd caffe && \
# Install python dependencies
cat python/requirements.txt | xargs -n1 pip install

echo "Pre-install: Done"
