## 1.Environment

- ubuntu 16.04 LTS
- git
- cmake
- ROS kinetic
- PCAP
- LCM

### 1.1.ROS kinetic
```
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116

    sudo apt-get update
    
    sudo apt-get install ros-kinetic-desktop-full
    
    sudo rosdep init
    
    rosdep update
    
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
   
    source ~/.bashrc
   
    sudo apt-get install python-rosinstall
    
    mkdir catkin_velodyne
```   

### 1.2.PCAP
```
    sudo apt-get install flex
    
    sudo apt-get install bison
    
    wget http://www.tcpdump.org/release/libpcap-1.4.0.tar.gz
    
    tar -zxvf libpcap-1.4.0.tar.gz 

    cd libpcap-1.4.0 

    sudo ./configure 

    sudo make 

    sudo make install
```

### 1.3.LCM
```
    git clone https://github.com/lcm-proj/lcm.git

    cmake .

    sudo make

    sudo make install

    sudo ldconfig
```

## 2.Running

Make sure the computer is connected to another collaborative mapping computer through wired or wireless channel. Keep only one Net Adapter so that all the lcm data could run through this device. Then, start terminal:

```
    roslaunch loam_velodyne loam_velodyne.launch
```
Start another terminal on this computer to play bag:
```
    rosbag play xxx.bag
```

## 3.Results

All results will be stored in the path:~/catkin_velodyne/data

- PCD

Global corner Maps and surf Maps whose origins are LiDAR's start position . Corner maps contains point cloud maps calculated from this car and other car(collaborative mapping result), while surf maps contains map from this car only.

- Origin PCD

Local corner and surf maps whose origins are LiDAR's current position.

- path.txt

Store this car LiDAR's odometry.

- thisCarGPS.txt

Store this car GPS data from IMU.

- thisCarGCL.txt

Store this car GCL data changed from GPS to show in the AMAP.

- thisCarGPSXY.txt

Store this car GPS to XY coodrinate.

- GPSXYFromOthers.txt

Store another car GPS to XY coodrinate.

- OdomXYFromOthers.txt

Store another car LiDAR odometry after RT.

- diff.txt

Store the matching init guess information.

- Matching

Store matched 2 maps from this car and another car in the same time.

## 4.Reference

[Install ROS Kinetic under ubuntu 16.04](http://blog.csdn.net/weicao1990/article/details/52575314)

[Install libpcap under ubuntu 16.04](http://blog.csdn.net/u011573853/article/details/49916025)
