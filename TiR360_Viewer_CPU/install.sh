#!/bin/sh
#
# Python 3.10 && OpenCV build with CUDA && Ubuntu 22.10

sudo apt-get install nlohmann-json3-dev
sudo apt-get install libyaml-cpp-dev
sudo apt-get install pkg-config
sudo apt-get install pybind11-dev
#$sudo apt install python3-pip
#$pip install pybind11

cmake -B build .
cmake --build build
