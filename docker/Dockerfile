# Dockerfile

FROM ubuntu:bionic

# essential build tools
RUN apt-get update \
 && apt-get install -y \
        cmake \
        pkg-config \
        git \
        build-essential \
        sudo \
        lsb-release \
        wget \
 && apt-get clean

# install prerequisites
RUN apt-get install -y \
	libglfw3-dev \
	libtinyxml2-dev \
	libeigen3-dev \
	libasound2-dev \
	libusb-1.0.0-dev \
	libxmu-dev \
	libxi-dev \
	libxrandr-dev \
	libxinerama-dev \
	libgl1-mesa-dev \
	libglu1-mesa-dev \
	libx11-dev \
	libxcursor-dev

# install chai3d
RUN git clone https://github.com/manips-sai-org/chai3d.git \
 && cd chai3d \
 && mkdir build \
 && cd build \
 && cmake .. -DCMAKE_BUILD_TYPE=Release \
 && make -j4

# install sai2urdfreader
RUN git clone https://github.com/manips-sai-org/sai2-urdfreader.git \
 && cd sai2-urdfreader \
 && mkdir build \
 && cd build \
 && cmake .. -DCMAKE_BUILD_TYPE=Release \
 && make -j4

# install sai2model
RUN git clone https://github.com/manips-sai-org/sai2-model.git \
 && cd sai2-model/rbdl \
 && mkdir build \
 && cd build \
 && cmake -DRBDL_BUILD_ADDON_URDFREADER=ON -DCMAKE_BUILD_TYPE=Release .. \
 && make -j4 \
 && cd ../.. \
 && mkdir build \
 && cd build \
 && cmake -DCMAKE_BUILD_TYPE=Release .. \
 && make -j4

# install sai2simulation dependencies (for examples actually)
RUN apt-get install -y \
	freeglut3-dev \
	libglew-dev