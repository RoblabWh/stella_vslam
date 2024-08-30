FROM nvcr.io/nvidia/cuda:12.4.0-devel-ubuntu22.04
ARG DEBIAN_FRONTEND=noninteractive

# Install build tools
RUN apt update && \
    apt upgrade -y && \
    apt install -y \
        lsb-release \
        wget \
        software-properties-common \
        gnupg \
        make \
        git \
        unzip \
        && \
    cd /tmp && \
    wget https://apt.llvm.org/llvm.sh && \
    bash llvm.sh 17 all && \
    wget -O cmake.sh https://github.com/Kitware/CMake/releases/download/v3.29.5/cmake-3.29.5-linux-x86_64.sh && \
    bash cmake.sh --prefix=/usr --skip-license && \
    rm -r /tmp/*

# Set compiler
ENV CC=clang-17
ENV CXX=clang++-17

# Build and install opencv
RUN apt install -y \
        python3-dev \
        python3-numpy \
        && \
    cd /tmp && \
    wget -O opencv.zip https://github.com/opencv/opencv/archive/4.10.0.zip && \
    wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.10.0.zip && \
    unzip opencv.zip && \
    unzip opencv_contrib.zip && \
    cd opencv-4.10.0 && \
    cmake -Bbuild -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.10.0/modules -DWITH_CUDA=ON -DCUDA_ARCH_BIN=native . && \
    cmake --build build --parallel $(nproc) && \
    cmake --install build && \
    rm -r /tmp/*

# Install stitcher dependencies
RUN apt install -y \
        nlohmann-json3-dev \
        libyaml-cpp-dev \
        pybind11-dev
