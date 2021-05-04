# realsense-ros2-lightwrapper
This repository contains light weight python ros2 wrapper for realsense sensors.

I made this code for the purpose of using L515's PointCloud2 data with ROS2 in Jetson Nano Developer Kit.

# Running pyrealsense2 in Jetson Nano Developer kit
Refer : [Running pyrealsense2 on JetsonNano](https://github.com/IntelRealSense/librealsense/issues/6964)

- Download librealsense2 code [here](https://github.com/IntelRealSense/librealsense/releases/) or [Direct Download link for v2.43.0](https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.43.0.zip)
```
# install dependencies and make gcc-8 for building with CUDA
sudo apt install -y libxinerama-dev libxcursor-dev gcc-8 g++-8
sudo ln -s /usr/bin/gcc-8 /usr/local/cuda-10.2/bin/gcc
sudo ln -s /usr/bin/g++-8 /usr/local/cuda-10.2/bin/g++

# unzip librealsense zip file
unzip librealsense-2.43.0
cd librealsense-2.43.0
mkdir build && cd build

cmake ../ -DFORCE_RSUSB_BACKEND=ON -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3
export CUDACXX=/usr/local/cuda-10.2/bin/nvcc

cmake ../ -DFORCE_RSUSB_BACKEND=ON -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DBUILD_WITH_CUDA:bool=true

# make and install
make -j4
sudo make install
```

You can check whether pyrealsense2 is work well or not by executing following script
```
python3
>>> import pyrealsense2.pyrealsense2 as rs
>>> rs.pipeline()
```

Finally, you can execute publisher as follows.
```
colcon build
ros2 run rs2_light_wrapper rs2_light_node

# To check the topic is working well,
ros2 topic hz /point_cloud
```
