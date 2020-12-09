# CooperativeMapping

Cooperative Mapping method based on LOAM and Segmap.



The computer should under the following environments:

Ubuntu 16.04 + ROS Kinetic

PCL-1.8

LCM-1.4.0 

Ceres Solver-2.0.0

Tensorflow-1.8

## 1&ensp;Dependencies

#### 1.1&ensp;Install ROS



#### 1.2&ensp;Install PCL

Install the required dependencies first

```
$ sudo apt-get update
$ sudo apt-get install git build-essential linux-libc-dev
$ sudo apt-get install cmake cmake-gui 
$ sudo apt-get install libusb-1.0-0-dev libusb-dev libudev-dev
$ sudo apt-get install mpi-default-dev openmpi-bin openmpi-common  
$ sudo apt-get install libflann1.8 libflann-dev
$ sudo apt-get install libeigen3-dev
$ sudo apt-get install libboost-all-dev
$ sudo apt-get install libvtk5.10-qt4 libvtk5.10 libvtk5-dev
$ sudo apt-get install libqhull* libgtest-dev
$ sudo apt-get install freeglut3-dev pkg-config
$ sudo apt-get install libxmu-dev libxi-dev 
$ sudo apt-get install mono-complete
$ sudo apt-get install qt-sdk openjdk-8-jdk openjdk-8-jre
```

Download the source code of PCL

```
$ git clone https://github.com/PointCloudLibrary/pcl.git
```

Compile source code

```
$ cd pcl
$ mkdir release
$ cd release
$ cmake -DCMAKE_BUILD_TYPE=None -DCMAKE_INSTALL_PREFIX=/usr \
           -DBUILD_GPU=ON -DBUILD_apps=ON -DBUILD_examples=ON \
           -DCMAKE_INSTALL_PREFIX=/usr ..
$ make
```

Then install it

```
$ sudo make install
$ sudo apt-get install ros-kinetic-pcl-conversions ros-kinect-pcl-ros
```

#### 1.3 &ensp; Install Ceres Solver



#### 1.4 &ensp; Install LCM

Install required dependencies

```
$ sudo apt-get install autoconf automake autopoint libglib2.0-dev libtool python-dev
```

Download *LCM* package from here 

```
$ unzip lcm-1.4.0.zip
$ cd lcm-1.4.0/
$ ./configure
$ sudo make
$ sudo make install
$ sudo ldconfig
```

#### 1.5 &ensp; Set up the workspace configuration

First install the required system packages:

```
$ sudo apt-get install python-wstool doxygen python3-pip python3-dev python-virtualenv dh-autoreconf
```

Set up the workspace configuration:

```
$ mkdir -p ~/catkin_velodyne/src
$ cd ~/catkin_velodyne
$ catkin init
$ catkin config --merge-devel
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Then use wstool for fetching catkin dependencies:

```
$ cd src
$ git clone https://github.com/sinetree/CooperativeMapping.git
$ wstool init
$ wstool merge CooperativeMapping/dependencies.rosinstall
$ wstool update
```

## 2 &ensp; Compilation

#### 2.1 &ensp; Build Tensorflow from source(more details can be seen here)

Install the TensorFlow *pip* package dependencies

```
$ pip install -U --user pip numpy wheel
$ pip install -U --user keras_preprocessing --no-deps
```

Install Bazel (There are different ways to insall Bazel, we use APT repository as an example)

```
$ sudo apt-get update && sudo apt-get install bazel
$ sudo apt-get upgrade bazel
```

Download the TensorFlow source code

```
$ git clone -b r1.8 https://github.com/tensorflow/tensorflow.git
$ cd tensorflow-r1.8
```

Configure the build by running the `./configure` at the root of your Tensorflow source tree.

```
$ ./configure
```

###### Sample session

The following shows a sample run of `./configure` script (your session may differ):

```
$ ./configure
You have bazel 3.0.0 installed.
Please specify the location of python. [Default is /usr/bin/python3]: 

Found possible Python library paths:
  /usr/lib/python3/dist-packages
  /usr/local/lib/python3.6/dist-packages
Please input the desired Python library path to use.  Default is [/usr/lib/python3/dist-packages]

Do you wish to build TensorFlow with OpenCL SYCL support? [y/N]: 
No OpenCL SYCL support will be enabled for TensorFlow.

Do you wish to build TensorFlow with ROCm support? [y/N]: 
No ROCm support will be enabled for TensorFlow.

Do you wish to build TensorFlow with CUDA support? [y/N]: Y
CUDA support will be enabled for TensorFlow.

Do you wish to build TensorFlow with TensorRT support? [y/N]: 
No TensorRT support will be enabled for TensorFlow.

Found CUDA 10.1 in:
    /usr/local/cuda-10.1/targets/x86_64-linux/lib
    /usr/local/cuda-10.1/targets/x86_64-linux/include
Found cuDNN 7 in:
    /usr/lib/x86_64-linux-gnu
    /usr/include


Please specify a list of comma-separated CUDA compute capabilities you want to build with.
You can find the compute capability of your device at: https://developer.nvidia.com/cuda-gpus Each capability can be specified as "x.y" or "compute_xy" to include both virtual and binary GPU code, or as "sm_xy" to only include the binary code.
Please note that each additional compute capability significantly increases your build time and binary size, and that TensorFlow only supports compute capabilities >= 3.5 [Default is: 3.5,7.0]: 6.1


Do you want to use clang as CUDA compiler? [y/N]: 
nvcc will be used as CUDA compiler.

Please specify which gcc should be used by nvcc as the host compiler. [Default is /usr/bin/gcc]: 


Please specify optimization flags to use during compilation when bazel option "--config=opt" is specified [Default is -march=native -Wno-sign-compare]: 


Would you like to interactively configure ./WORKSPACE for Android builds? [y/N]: 
Not configuring the WORKSPACE for Android builds.

Preconfigured Bazel build configs. You can use any of the below by adding "--config=<>" to your build command. See .bazelrc for more details.
    --config=mkl            # Build with MKL support.
    --config=monolithic     # Config for mostly static monolithic build.
    --config=ngraph         # Build with Intel nGraph support.
    --config=numa           # Build with NUMA support.
    --config=dynamic_kernels    # (Experimental) Build kernels into separate shared objects.
    --config=v2             # Build TensorFlow 2.x instead of 1.x.
Preconfigured Bazel build configs to DISABLE default on features:
    --config=noaws          # Disable AWS S3 filesystem support.
    --config=nogcp          # Disable GCP support.
    --config=nohdfs         # Disable HDFS support.
    --config=nonccl         # Disable NVIDIA NCCL support.
Configuration finished
```

Build the pip package

```
$ bazel build --config=opt --define framework_shared_object=false
tensorflow:libtensorflow_cc.so
```

#### 2.2 &ensp; Build tensorflow_ros_cpp

Find the CMakeLists.txt file in *tensorflow_ros_cpp* package and set up the path, 

```
set(TF_BAZEL_LIBRARY “CATKIN_DEVEL_PREFIX/…/libtensorflow_cc.so” CACHE STRING “Path to the bazel-compiled Tensorflow C++ library”)
set(TF_BAZEL_SRC_DIR “${CATKIN_DEVEL_PREFIX}/…/tensorflow-include-base” CACHE STRING “Path to the Tensorflow sources directory”)
```

modify the library path and source path above to the paths determined in step 2.1

For example,

```
set(TF_BAZEL_LIBRARY “/home/ubuntu/tensorflow-r1.8/bazel-bin/tensorflow/libtensorflow_cc.so” CACHE STRING “Path to the bazel-compiled Tensorflow C++ library”)
set(TF_BAZEL_SRC_DIR “/home/ubuntu/tensorflow-r1.8” CACHE STRING “Path to the Tensorflow sources directory”)
```

Then compile the tensorflow_ros_cpp package

```
$ cd ~/catkin_velodyne
$ catkin build tensorflow_ros_cpp
```

#### 2.3 &ensp; Build loam_velodyne

Install some packages first：

```
$ sudo apt-get install autoconf automake libtool
```

```
$ git clone https://github.com/doxygen/doxygen.git
$ cd doxygen
$ mkdir build
$ cd build
$ cmake …
$ make
$ sudo make install
```

```
$ git clone https://github.com/ros-perception/pcl_conversions/tree/indigo-devel
$ cd pcl_conversions
$ mkdir build
$ cd build
$ cmake …
$ make
$ sudo make install
```

Finally, build the *loam_velodyne* package which will compile all dependencies and SegMap modules:

```
$ cd ~/catkin_velodyne
$ catkin build loam_velodyne
```

## 3 &ensp; Running 

Make sure to source the workspace before running:

```
$ source ~/catkin_velodyne/devel/setup.bash
```

#### 3.1 &ensp; Download demonstration files

Download the segmap data from [here](链接:https://pan.baidu.com/s/10ZmnXQMdqTU8qm3SRI1XqQ  密码:gscv). 

#### 3.2 &ensp; Run examples

An online cooperative example can be run with

```
$ roslaunch loam_velodyne loam_velodyne_HDL64.launch
```

In the second terminal play sample velodyne data from kitti rosbag:

```
$ rosbag play xxx.bag 
```

