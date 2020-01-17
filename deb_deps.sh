#! /bin/bash
#
# Install the system dependencies required to build TerrainRL
#
apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler -y
apt-get install --no-install-recommends libboost-all-dev -y
apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev -y
apt-get install libatlas-base-dev -y
apt-get install gcc-4.9-multilib g++-4.9-multilib -y
apt-get install libf2c2-dev -y
apt-get install libglew-dev freeglut3 freeglut3-dev -y
apt-get install premake4 -y
## This is not necessary
#  sudo apt-get install cuda
#  sudo apt-get install swig3.0 python3-dev python3-pip -y
