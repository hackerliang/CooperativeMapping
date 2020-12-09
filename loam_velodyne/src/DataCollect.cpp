#include "DataCollect.h"
#include <cmath>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_plotter.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

double eePara = 0.00669342162296594323;
double aaPara = 6378245.0;
/** @brief Funtcions for GPS -> GCJ axis transformation
**/
double transformLat(double x, double y) {
    double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * sqrt(x > 0 ? x : -x);
    ret += (20.0 * sin(6.0 * x * M_PI) + 20.0 * sin(2.0 * x * M_PI)) * 2.0 / 3.0;
    ret += (20.0 * sin(y * M_PI) + 40.0 * sin(y / 3.0 * M_PI)) * 2.0 / 3.0;
    ret += (160.0 * sin(y / 12.0 * M_PI) + 320 * sin(y * M_PI / 30.0)) * 2.0 / 3.0;
    return ret;
}


/** @brief Funtcions for GPS -> GCJ axis transformation
**/
double transformLon(double x, double y) {
    double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(x > 0 ? x : -x);
    ret += (20.0 * sin(6.0 * x * M_PI) + 20.0 * sin(2.0 * x * M_PI)) * 2.0 / 3.0;
    ret += (20.0 * sin(x * M_PI) + 40.0 * sin(x / 3.0 * M_PI)) * 2.0 / 3.0;
    ret += (150.0 * sin(x / 12.0 * M_PI) + 300.0 * sin(x / 30.0 * M_PI)) * 2.0 / 3.0;
    return ret;
}


/*  @brief Call this funciton to get the corrected longitude and latitude
    @param Longitude in WGS-84 to be converted
    @param Latitude in WGS-84 to be converted
    @param New Longitude in GCJ
    @param New Latitude in GCT

    example: 
    double lng, lat;//get from GPS
    double new_lng, new_lat;//to hold transformed data
    transformFromWGSToGCJ(lng, lat, &new_lng, &new_lat)
*/
void transformFromWGSToGCJ(double longitude, double latitude, double *newLng, double *newLat) {
    double dLat = transformLat(longitude - 105.0, latitude - 35.0);
    double dLon = transformLon(longitude - 105.0, latitude - 35.0);
    double radLat = latitude / 180.0 * M_PI;
    double magic = sin(radLat);
    magic = 1 - eePara * magic * magic;
    double sqrtMagic = sqrt(magic);
    dLat = (dLat * 180.0) / ((aaPara * (1 - eePara)) / (magic * sqrtMagic) *  M_PI);
    dLon = (dLon * 180.0) / (aaPara / sqrtMagic *  cos(radLat) *  M_PI);
    *newLng = dLon + longitude;
    *newLat = dLat + latitude;
}


/** @brief Funtcions for calculate 2 GPS points disance, return exclidean distance
    @param this_x one point x axis
    @param this_y one point y axis
    @param other_x another point x axis
    @param other_y another point y axis 
**/
double getEuclideanDistance(double this_x, double this_y, double other_x, double other_y) {
	return sqrt((this_x-other_x)*(this_x-other_x) +(this_y-other_y)*(this_y-other_y));
}


/** @brief Funtcions for calculate 2 lidar odom points disance, return exclidean distance
    @param x1 one point x axis
    @param y1 one point y axis
    @param z1 one point z axis
    @param x2 another point x axis
    @param y2 another point y axis 
    @param z2 another point z axis
**/
double getEuclideanDistance(double x1, double y1, double z1, double x2, double y2, double z2) {
	return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));
}


/** @brief Funtcions transform two cars heading difference from angle to radian
    @param headingAngle heading difference angle
    @return heading difference radian
**/
double getHeadingDifferenceRadian(double headingAngle) {
    return ((M_PI * headingAngle) / 180.0);
}

/** @brief Funtcions rotate gps coordinate, because gps to xy is 45 degree different to loam coordinate
    @param x, y gps coordinate before
    @param x_after, y_after gps coordinate after
**/
void rotateGPSCoordinate(double x, double y, double* x_after, double* y_after, double x_diff, double y_diff, double heading_diff) {
    Eigen::Matrix3f rotate = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f translate = Eigen::Matrix3f::Identity();
    Eigen::Vector3f GPSPosition = Eigen::Vector3f::Zero();
    float rotate_coordinate = getHeadingDifferenceRadian(heading_diff-90);
    rotate(0, 0) = cos(rotate_coordinate);
    rotate(0, 1) = -sin(rotate_coordinate);
    rotate(0, 2) = 0;
    rotate(1, 0) = sin(rotate_coordinate);
    rotate(1, 1) = cos(rotate_coordinate);
    rotate(1, 2) = 0;

    translate(0, 2) = x_diff;
    translate(1, 2) = y_diff;
    
    GPSPosition(0) = x;
    GPSPosition(1) = y;
    GPSPosition(2) = 1;
    
    GPSPosition = rotate * translate * GPSPosition;

    *x_after = GPSPosition(0);
    *y_after = GPSPosition(1);
}