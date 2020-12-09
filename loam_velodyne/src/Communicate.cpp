#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "DataCollect.h"
#include <loam_velodyne/common.h>
#include <nav_msgs/Odometry.h>
#include "lcm/lcm-cpp.hpp"
#include "lcm/lcm.h"
#include "Communicate/Location.hpp"
#include "Communicate/PointCloud.hpp"
#include "Communicate/Pose.hpp"
#include "Communicate/MatchingResult.hpp"
#include "Geography/Geography.hpp"
#include "Geography/GaussLocalGeographicCS.hpp"
#include "../include/novatel_msgs/INSPVAX.h"
#include "../include/loam_velodyne_msg/Heading.h"
#include "../include/loam_velodyne_msg/NodeTransform.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_plotter.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <pthread.h>

#define EUCLIDEAN_DISTANCE_THRESHOLD 1.5

using namespace std;

void sendMapPackage(Communicate::PointCloud data);
void sendMaprvizPackage(Communicate::PointCloud data);
void sendPosePackage(Communicate::Pose data);

void* threadReceiveMapFunction(void* param);
void* threadReceivePoseFunction(void* param);

lcm::LCM receiveMapLCM("udpm://239.255.76.63:7667?ttl=1");

lcm::LCM receivePoseLCM("udpm://239.255.76.63:7667?ttl=1");

void mapToOthersHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudPoint);
void poseToOthersHandler(const nav_msgs::Odometry& odom);

pcl::PointCloud<PointType>::Ptr sendMapToOthers(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr receiveMapFromOthers(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr sendCloudToOthers(new pcl::PointCloud<PointType>());

long counter = 0;
long sendPacketCounter = 0;
long receivePacketCounter = 0;

ofstream sendPacket("/home/xyz/segmap/data/AnalysisNetwork/sendpacket.txt", ios::trunc);
ofstream receivePacket("/home/xyz/segmap/data/AnalysisNetwork/receivepacket.txt", ios::trunc);

vector<nav_msgs::Odometry::ConstPtr> thisCarPose;
vector<sensor_msgs::PointCloud2::ConstPtr> thisCarMap;

ros::Publisher *pubPoseFromOthersPointer = NULL;
ros::Publisher *pubMapFromOthersBeforeRTPointer = NULL;
ros::Publisher *pubMapFromOthersForRTPointer = NULL;

class receiveMapHandler {
public:
	~receiveMapHandler(){};
	void handleMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const Communicate::PointCloud* msg) {
		receiveMapFromOthers->clear();
		vector<vector<double> >point = msg->points;
		for (int i = 0; i < msg->num_ranges; i++) {
			PointType pointTemp;
			pointTemp.x = point[i][0];
			pointTemp.y = point[i][1];
			pointTemp.z = point[i][2];
			pointTemp.intensity = point[i][3];
			receiveMapFromOthers->push_back(pointTemp);
		}
        sensor_msgs::PointCloud2 mapOut;
		pcl::toROSMsg(*receiveMapFromOthers, mapOut);
        mapOut.header.stamp.fromNSec(msg->stamp);
        mapOut.header.frame_id = "/camera_init";
        pubMapFromOthersBeforeRTPointer->publish(mapOut);
        // receivePacketCounter++;
		// ros::Time startReceiving = ros::Time::now();
		// receivePacket << "NO." << receivePacketCounter << ", " << startReceiving << std::endl;
	}
};



class receivePoseHandler {
public:
	~receivePoseHandler(){};
	void handleMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const Communicate::Pose* msg) {
		nav_msgs::Odometry odom;
		odom.header.stamp.fromNSec(msg->stamp);
		odom.header.frame_id = "/camera_init";
		odom.child_frame_id = "/camera";
		odom.pose.pose.orientation.x = msg->orientationX;
  		odom.pose.pose.orientation.y = msg->orientationY;
  		odom.pose.pose.orientation.z = msg->orientationZ;
  		odom.pose.pose.orientation.w = msg->orientationW;
  		odom.pose.pose.position.x = msg->positionX;
  		odom.pose.pose.position.y = msg->positionY;
  		odom.pose.pose.position.z = msg->positionZ;
  	 	pubPoseFromOthersPointer->publish(odom);
	}
};


void mapToOthersHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudPointIn) {

	sendMapToOthers->clear();
	pcl::fromROSMsg(*laserCloudPointIn, *sendMapToOthers);
	Communicate::PointCloud data;
    data.stamp = laserCloudPointIn->header.stamp.toNSec();
	data.id = counter;
	data.num_ranges = (int)sendMapToOthers->size();
	for (int i = 0; i < sendMapToOthers->size();i++) {		
		vector<double> point;
		point.push_back(sendMapToOthers->points[i].x);
		point.push_back(sendMapToOthers->points[i].y);
		point.push_back(sendMapToOthers->points[i].z);
		point.push_back(sendMapToOthers->points[i].intensity);
		data.points.push_back(point);
	}
	sendMapPackage(data);
	counter++;
}


void poseToOthersHandler(const nav_msgs::Odometry::ConstPtr& odom) {

    Communicate::Pose data;
	data.id = counter;
	data.stamp = odom->header.stamp.toNSec();
	data.orientationX = odom->pose.pose.orientation.x;
	data.orientationY = odom->pose.pose.orientation.y;
	data.orientationZ = odom->pose.pose.orientation.z;
	data.orientationW = odom->pose.pose.orientation.w;
	data.positionX = odom->pose.pose.position.x;
	data.positionY = odom->pose.pose.position.y;
	data.positionZ = odom->pose.pose.position.z;
	sendPosePackage(data);
	//sendPacketCounter++;
	//sendPacket << "NO." << sendPacketCounter << ", " << startSending << std::endl;
	counter++;
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "communicate");
	ros::NodeHandle nh;

	ros::Publisher pubPoseFromOthers = nh.advertise<nav_msgs::Odometry> ("/pose_from_others", 5);

	ros::Publisher pubMapFromOthersBeforeRT = nh.advertise<sensor_msgs::PointCloud2> ("/map_for_rt_from_others", 5);

	ros::Subscriber subMapToOthers = nh.subscribe<sensor_msgs::PointCloud2> ("/map_to_others", 5, mapToOthersHandler);
	
	ros::Subscriber subPoseToOthers = nh.subscribe<nav_msgs::Odometry> ("/aft_mapped_to_init", 5, poseToOthersHandler);

	pubPoseFromOthersPointer = &pubPoseFromOthers;

	pubMapFromOthersBeforeRTPointer = &pubMapFromOthersBeforeRT;
	
	pthread_t threadForReceiveMap;
	
	pthread_t threadForReceivePose;

	pthread_create(&threadForReceiveMap, NULL, threadReceiveMapFunction, NULL);
	
	pthread_create(&threadForReceivePose, NULL, threadReceivePoseFunction, NULL);

	receiveMapHandler rmh;
	receiveMapLCM.subscribe("MapCar1", &receiveMapHandler::handleMessage, &rmh);

	receivePoseHandler rph;
	receivePoseLCM.subscribe("PoseCar1", &receivePoseHandler::handleMessage, &rph);


	ros::Rate rate(100);
	bool status = ros::ok();
	while(status) {
       ros::Time begin = ros::Time::now();
	   ros::spinOnce();
	   status = ros::ok();
       rate.sleep();    	
	}
	return 0;
}

void sendMapPackage(Communicate::PointCloud data) {
	lcm::LCM sendMapLCM("udpm://239.255.76.63:7667?ttl=1");
	if(!sendMapLCM.good()) {
		printf("lcm is bad\n");
		return;
	}
	sendMapLCM.publish("MapCar2", &data);
}


void sendPosePackage(Communicate::Pose data) {
	lcm::LCM sendPoseLCM("udpm://239.255.76.63:7667?ttl=1");
	if(!sendPoseLCM.good()) {
		printf("lcm is bad\n");
		return;
	}
	sendPoseLCM.publish("PoseCar2", &data);
}


void* threadReceiveMapFunction(void* param) {
	while(0 == receiveMapLCM.handle()) {
		usleep(10000);
	}
}

void* threadReceivePoseFunction(void* param) {
	while(0 == receivePoseLCM.handle()) {
		usleep(10000);
	}
}
