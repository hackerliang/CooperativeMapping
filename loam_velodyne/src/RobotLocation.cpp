
#include <cmath>

#include <loam_velodyne/common.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <boost/format.hpp>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#define POINTTYPE PointType
using namespace std;

pcl::PointCloud<POINTTYPE>::Ptr CurrentCornerFrame(
    new pcl::PointCloud<POINTTYPE>());
pcl::PointCloud<POINTTYPE>::Ptr CurrentSurfFrame(
    new pcl::PointCloud<POINTTYPE>());
pcl::PointCloud<POINTTYPE>::Ptr CurrentCornerinWorld(
    new pcl::PointCloud<POINTTYPE>());
pcl::PointCloud<POINTTYPE>::Ptr CurrentSurfinWorld(
    new pcl::PointCloud<POINTTYPE>());

pcl::PointCloud<POINTTYPE>::Ptr MapPointsbuilld(new pcl::PointCloud<POINTTYPE>());
pcl::PointCloud<POINTTYPE>::Ptr imuTrans(new pcl::PointCloud<POINTTYPE>());

pcl::PointCloud<POINTTYPE>::Ptr laserCloudCornerLast(
    new pcl::PointCloud<POINTTYPE>());
pcl::PointCloud<POINTTYPE>::Ptr laserCloudSurfLast(
    new pcl::PointCloud<POINTTYPE>());

pcl::PointCloud<POINTTYPE>::Ptr laserCloudOri(new pcl::PointCloud<POINTTYPE>());
pcl::PointCloud<POINTTYPE>::Ptr coeffSel(new pcl::PointCloud<POINTTYPE>());

pcl::KdTreeFLANN<POINTTYPE>::Ptr kdtreeCornerFromMap(
    new pcl::KdTreeFLANN<POINTTYPE>());
pcl::KdTreeFLANN<POINTTYPE>::Ptr kdtreeSurfFromMap(
    new pcl::KdTreeFLANN<POINTTYPE>());
pcl::KdTreeFLANN<POINTTYPE>::Ptr MapPointSearch(
    new pcl::KdTreeFLANN<POINTTYPE>());

pcl::PointCloud<pcl::Normal>::Ptr cloudNormals_cur(
    new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::Normal>::Ptr cloudNormals_map(
    new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_cur(
    new pcl::PointCloud<pcl::FPFHSignature33>());
pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_map(
    new pcl::PointCloud<pcl::FPFHSignature33>());
pcl::PointCloud<POINTTYPE>::Ptr object_aligned(
    new pcl::PointCloud<POINTTYPE>());
typedef pcl::visualization::PointCloudColorHandlerCustom<
    pcl::PointCloud<POINTTYPE> > ColorHandlerT;
pcl::PointCloud<POINTTYPE>::Ptr pairalignoutput(
    new pcl::PointCloud<POINTTYPE>());
Eigen::Matrix4f final_transform;
int globaltimes = 0;

double timeCurrentCornerFrame;
double timeCurrentSurfFrame;
double timeImuTrans;

bool newCurrentCornerFrame = 0;
bool newCurrentSurfFrame = 0;
bool newImuTrans = 0;

int laserCloudCornerLastNum;
int laserCloudSurfLastNum;
int pathNum = 5;

POINTTYPE RobotLocation;

/* int pointSelCornerInd[40000]; */
float pointSearchCornerInd1[20000];
float pointSearchCornerInd2[20000];

/* int pointSelSurfInd[40000]; */
float pointSearchSurfInd1[20000];
float pointSearchSurfInd2[20000];
float pointSearchSurfInd3[20000];
float ScanToMap[6] = {0};
float Robottransform[6] = {0};
float transformAftMapped[6] = {0};

void LidarpointToWorld(POINTTYPE const* const pi, POINTTYPE*  po) {
  // cout << "start lidar point to world" << endl;
  float x1 = cos(ScanToMap[2]) * pi->x - sin(ScanToMap[2]) * pi->y;
  float y1 = sin(ScanToMap[2]) * pi->x + cos(ScanToMap[2]) * pi->y;
  float z1 = pi->z;

  float x2 = x1;
  float y2 = cos(ScanToMap[0]) * y1 - sin(ScanToMap[0]) * z1;
  float z2 = sin(ScanToMap[0]) * y1 + cos(ScanToMap[0]) * z1;
  // cout << "start setting point po" << endl;
  po->x = cos(ScanToMap[1]) * x2 + sin(ScanToMap[1]) * z2 + ScanToMap[3];
  po->y = y2 + ScanToMap[4];
  // cout << "setting point po" << endl;
  po->z = -sin(ScanToMap[1]) * x2 + cos(ScanToMap[1]) * z2 + ScanToMap[5];
  po->intensity = pi->intensity;
  // cout << "finish lidar point to world" << endl;
}

void TransformToStart(POINTTYPE const* const pi, POINTTYPE*  po) {
  float rx = Robottransform[0];
  float ry = Robottransform[1];
  float rz = Robottransform[2];
  float tx = Robottransform[3];
  float ty = Robottransform[4];
  float tz = Robottransform[5];

  float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
  float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
  float z1 = (pi->z - tz);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  po->x = cos(ry) * x2 - sin(ry) * z2;
  po->y = y2;
  po->z = sin(ry) * x2 + cos(ry) * z2;
  po->intensity = pi->intensity;
}

void laserCloudLessSharpHandler(
    const sensor_msgs::PointCloud2ConstPtr& CurrentCornerFrame2) {
  timeCurrentCornerFrame = CurrentCornerFrame2->header.stamp.toSec();

  CurrentCornerFrame->clear();
  CurrentCornerinWorld->clear();
  pcl::fromROSMsg(*CurrentCornerFrame2, *CurrentCornerFrame);
  pcl::fromROSMsg(*CurrentCornerFrame2, *CurrentCornerinWorld);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*CurrentCornerFrame, *CurrentCornerFrame,
                               indices);
  std::vector<int> indices2;
  pcl::removeNaNFromPointCloud(*CurrentCornerinWorld, *CurrentCornerinWorld,
                               indices2);
  globaltimes++;
                               
  newCurrentCornerFrame = true;
}

void laserCloudLessFlatHandler(
    const sensor_msgs::PointCloud2ConstPtr& CurrentSurfFrame2) {
  timeCurrentSurfFrame = CurrentSurfFrame2->header.stamp.toSec();

  CurrentSurfFrame->clear();
  CurrentSurfinWorld->clear();
  pcl::fromROSMsg(*CurrentSurfFrame2, *CurrentSurfFrame);
  pcl::fromROSMsg(*CurrentSurfFrame2, *CurrentSurfinWorld);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*CurrentSurfFrame, *CurrentSurfFrame, indices);
  std::vector<int> indices2;
  pcl::removeNaNFromPointCloud(*CurrentSurfinWorld, *CurrentSurfinWorld,
                               indices2);
  newCurrentSurfFrame = true;
}

void imuTransHandler(const sensor_msgs::PointCloud2ConstPtr& imuTrans2) {
  timeImuTrans = imuTrans2->header.stamp.toSec();

  imuTrans->clear();
  pcl::fromROSMsg(*imuTrans2, *imuTrans);

  newImuTrans = true;
}

void IMUOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped) {
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w))
      .getRPY(roll, pitch, yaw);

  transformAftMapped[0] = -pitch;
  transformAftMapped[1] = -yaw;
  transformAftMapped[2] = roll;

  transformAftMapped[3] = odomAftMapped->pose.pose.position.x;
  transformAftMapped[4] = odomAftMapped->pose.pose.position.y;
  transformAftMapped[5] = odomAftMapped->pose.pose.position.z;
}

void computeNormals(pcl::PointCloud<POINTTYPE>::Ptr cloud,
                    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals) {
  pcl::NormalEstimation<POINTTYPE, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  pcl::search::KdTree<POINTTYPE>::Ptr tree(
      new pcl::search::KdTree<POINTTYPE>());
  ne.setSearchMethod(tree);
  // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new
  // pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch(0.03);
  ne.compute(*cloud_normals);
}

void computeFeatures(pcl::PointCloud<POINTTYPE>::Ptr cloud,
                     pcl::PointCloud<pcl::Normal>::Ptr normals,
                     pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs) {
  pcl::FPFHEstimation<POINTTYPE, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud(cloud);
  fpfh.setInputNormals(normals);
  pcl::search::KdTree<POINTTYPE>::Ptr tree(new pcl::search::KdTree<POINTTYPE>);
  fpfh.setSearchMethod(tree);
  fpfh.setRadiusSearch(0.05);
  fpfh.compute(*fpfhs);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "RobotLocation");
  ros::NodeHandle nh;

  ros::Subscriber subCurrentCornerFrame =
      nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 2,
                                             laserCloudLessSharpHandler);

  ros::Subscriber subCurrentSurfFrame = nh.subscribe<sensor_msgs::PointCloud2>(
      "/laser_cloud_less_flat", 2, laserCloudLessFlatHandler);

  ros::Subscriber subImuTrans =
      nh.subscribe<sensor_msgs::PointCloud2>("/imu_trans", 5, imuTransHandler);
  ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>(
      "/laser_odom_to_init", 5, IMUOdometryHandler);

  ros::Publisher pubLaserOdometry =
      nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 5);
  nav_msgs::Odometry laserOdometry;
  laserOdometry.header.frame_id = "/camera_init";
  laserOdometry.child_frame_id = "/laser_odom";

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform laserOdometryTrans;
  laserOdometryTrans.frame_id_ = "/camera_init";
  laserOdometryTrans.child_frame_id_ = "/laser_odom";
  std::vector<int> pointSearchInd(1);
  std::vector<float> pointSearchSqDis(1);

  POINTTYPE pointOri, pointSel, tripod1, tripod2, tripod3, pointProj, coeff;

  cv::Mat matA0(5, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matB0(5, 1, CV_32F, cv::Scalar::all(-1));
  cv::Mat matX0(3, 1, CV_32F, cv::Scalar::all(0));

  cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

  bool isDegenerate = false;
  cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

  pcl::VoxelGrid<POINTTYPE> downSizeFilterCorner;
  downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);

  pcl::VoxelGrid<POINTTYPE> downSizeFilterSurf;
  downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);

  pcl::VoxelGrid<POINTTYPE> downSizeFilterMap;
  downSizeFilterMap.setLeafSize(0.6, 0.6, 0.6);

  boost::format fmt("/media/es/beta/velodyne/PCD/%s%d.%s");
  boost::format fmt1("/media/es/beta/velodyne/path.pcd");
  pcl::io::loadPCDFile<POINTTYPE>(fmt1.str(), *MapPointsbuilld);
  cout << "loading mapponit" << endl;
  MapPointSearch->setInputCloud(MapPointsbuilld);
  cout << "loaded mapponit" << endl;

  ros::Rate rate(100);
  bool status = ros::ok();
  int ggtimes = 1;
  if (ggtimes < MapPointsbuilld->points.size()) {
    int ps = rand() % ggtimes;
    cout << "it's ps: " << ps << endl;
    RobotLocation = MapPointsbuilld->points[20];
    // RobotLocation.x += 0.0005f;
  }
  while (status) {
    /* cout<<"status ok"<<endl; */

    ros::spinOnce();
    // if(ggtimes<MapPoint->points.size()){
    //   int ps = rand()%ggtimes;
    //   cout<<"it's ps: "<<ps<<endl;
    //   RobotLocation = MapPoint->points[ps];
    //   }

    if (newCurrentCornerFrame && newCurrentSurfFrame && newImuTrans &&
        fabs(timeCurrentCornerFrame - timeCurrentSurfFrame) < 0.005 &&
        fabs(timeImuTrans - timeCurrentSurfFrame) < 0.005) {
      newCurrentCornerFrame = false;
      newCurrentSurfFrame = false;
      newImuTrans = false;
      int closestPointInd = -1;
      cout << "ggtimes: " << ggtimes++ << endl;
      /*  */
      // cout << "before map kn search" << endl;
      MapPointSearch->nearestKSearch(RobotLocation, 1, pointSearchInd,
                                     pointSearchSqDis);
      cout << "after search" << endl;
      cout << "pointSearchInd: " << pointSearchInd[0] << endl;
      cout << "pointSearchSqDis: " << pointSearchSqDis[0] << endl;
      if (pointSearchSqDis[0] < 50) closestPointInd = pointSearchInd[0];
      if (closestPointInd >= 0) {
        pcl::io::loadPCDFile<POINTTYPE>(
            (fmt % "Corner-" % (closestPointInd) % "pcd").str(),
            *laserCloudCornerLast);
        pcl::io::loadPCDFile<POINTTYPE>(
            (fmt % "Surf-" % (closestPointInd) % "pcd").str(),
            *laserCloudSurfLast);
        kdtreeCornerFromMap->setInputCloud(laserCloudCornerLast);
        kdtreeSurfFromMap->setInputCloud(laserCloudSurfLast);
      }
      // cout << "after corner, surf loaded" << endl;
      int laserCloudCornerLastNum = laserCloudCornerLast->points.size();
      int laserCloudSurfLastNum = laserCloudSurfLast->points.size();
      if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*CurrentCornerFrame, *CurrentCornerFrame,
                                     indices);

        int CurrentCornerNum = CurrentCornerFrame->points.size();
        int CurrentSurfNum = CurrentSurfFrame->points.size();
        // cout << "before corner tranform to world" << endl;
        /* transform lidar pointcloud to world reference frame */
        for (int i = 0; i < CurrentCornerNum; i++) {
          // cout << "CurrentCornerNum: " << CurrentCornerNum<<" now: "<<i <<
          // endl;

          LidarpointToWorld(&CurrentCornerFrame->points[i],
                            &CurrentCornerinWorld->points[i]);
        }
        for (int i = 0; i < CurrentSurfNum; i++) {
          // cout << "CurrentSurfNum: " << CurrentSurfNum<<" now: "<<i << endl;

          LidarpointToWorld(&(CurrentSurfFrame->points[i]),
                            &(CurrentSurfinWorld->points[i]));
        }

        // for(int i =0;i<laserCloudCornerLastNum;i++){
        //   cout<<CurrentSurfinWorld->points[i]<<"
        //   "<<laserCloudCornerLast->points[i]<<endl;
        //   cout<<((CurrentSurfinWorld->points[i].x==laserCloudCornerLast->points[i].x)?"true":"false")<<endl;
        // }
        // cout << "after surf to world" << endl;
        ///////
        // computeNormals(CurrentSurfinWorld, cloudNormals_cur);
        // computeNormals(laserCloudSurfLast, cloudNormals_map);
        // computeFeatures(CurrentSurfinWorld, cloudNormals_cur, fpfhs_cur);
        // computeFeatures(laserCloudSurfLast, cloudNormals_map, fpfhs_map);

        // // Perform alignment
        // pcl::console::print_highlight ("Starting alignment...\n");
        // pcl::SampleConsensusPrerejective<POINTTYPE,POINTTYPE,pcl::FPFHSignature33>
        // align;
        // align.setInputSource (CurrentSurfinWorld);
        // align.setSourceFeatures (fpfhs_cur);
        // align.setInputTarget (laserCloudSurfLast);
        // align.setTargetFeatures (fpfhs_map);
        // align.setMaximumIterations (100000); // Number of RANSAC iterations
        // align.setNumberOfSamples (3); // Number of points to sample for
        // generating/prerejecting a pose
        // align.setCorrespondenceRandomness (5); // Number of nearest features
        // to use
        // align.setSimilarityThreshold (0.9f); // Polygonal edge length
        // similarity threshold
        // align.setMaxCorrespondenceDistance (2.5f * 0.0005f); // Inlier
        // threshold
        // align.setInlierFraction (0.25f); // Required inlier fraction for
        // accepting a pose hypothesis
        // {
        //   pcl::ScopeTime t("Alignment");
        //   align.align (*object_aligned);
        // }

        // if (align.hasConverged ())
        // {
        //   // Print results
        //   printf ("\n");
        //   Eigen::Matrix4f transformation = align.getFinalTransformation ();
        //   pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n",
        //   transformation (0,0), transformation (0,1), transformation (0,2));
        //   pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n",
        //   transformation (1,0), transformation (1,1), transformation (1,2));
        //   pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n",
        //   transformation (2,0), transformation (2,1), transformation (2,2));
        //   pcl::console::print_info ("\n");
        //   pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n",
        //   transformation (0,3), transformation (1,3), transformation (2,3));
        //   pcl::console::print_info ("\n");
        //   pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers
        //   ().size (), CurrentCornerinWorld->size ());

        //   cout<<"success"<<endl;
        //   return 0;
        // }
        // else
        // {
        //   pcl::console::print_error ("Alignment failed!\n");

        // }
        ///////

        for (int iterCount = 0; iterCount < 100; iterCount++) {
          laserCloudOri->clear();
          coeffSel->clear();
          // cout<<"itercount: "<<iterCount<<endl;
          for (int i = 0; i < CurrentCornerNum; i++) {
            // cout<<"before to start"<<endl;
            TransformToStart(&CurrentCornerinWorld->points[i], &pointSel);
            // cout<<"after started"<<endl;
            if (iterCount % 5 == 0) {
              std::vector<int> indices;
              pcl::removeNaNFromPointCloud(*laserCloudCornerLast,
                                           *laserCloudCornerLast, indices);
              cout<<"pointsel: "<<pointSel<<endl;
              int ssresults1 = kdtreeCornerFromMap->nearestKSearch(pointSel, 1, pointSearchInd,
                                                  pointSearchSqDis);
              // cout<<"after cornerfrommap knsearch"<<endl;

              int closestPointInd = -1, minPointInd2 = -1;
              if (pointSearchSqDis[0] < 25) {
                closestPointInd = pointSearchInd[0];
                int closestPointScan = int(
                    laserCloudCornerLast->points[closestPointInd].intensity);

                float pointSqDis, minPointSqDis2 = 25;
                for (int j = closestPointInd + 1; j < CurrentCornerNum; j++) {
                  if (int(laserCloudCornerLast->points[j].intensity) >
                      closestPointScan + 2.5) {
                    break;
                  }

                  pointSqDis =
                      (laserCloudCornerLast->points[j].x - pointSel.x) *
                          (laserCloudCornerLast->points[j].x - pointSel.x) +
                      (laserCloudCornerLast->points[j].y - pointSel.y) *
                          (laserCloudCornerLast->points[j].y - pointSel.y) +
                      (laserCloudCornerLast->points[j].z - pointSel.z) *
                          (laserCloudCornerLast->points[j].z - pointSel.z);

                  if (int(laserCloudCornerLast->points[j].intensity) >
                      closestPointScan) {
                    if (pointSqDis < minPointSqDis2) {
                      minPointSqDis2 = pointSqDis;
                      minPointInd2 = j;
                    }
                  }
                }
                for (int j = closestPointInd - 1; j >= 0; j--) {
                  if (int(laserCloudCornerLast->points[j].intensity) <
                      closestPointScan - 2.5) {
                    break;
                  }

                  pointSqDis =
                      (laserCloudCornerLast->points[j].x - pointSel.x) *
                          (laserCloudCornerLast->points[j].x - pointSel.x) +
                      (laserCloudCornerLast->points[j].y - pointSel.y) *
                          (laserCloudCornerLast->points[j].y - pointSel.y) +
                      (laserCloudCornerLast->points[j].z - pointSel.z) *
                          (laserCloudCornerLast->points[j].z - pointSel.z);

                  if (int(laserCloudCornerLast->points[j].intensity) <
                      closestPointScan) {
                    if (pointSqDis < minPointSqDis2) {
                      minPointSqDis2 = pointSqDis;
                      minPointInd2 = j;
                    }
                  }
                }
              }

              pointSearchCornerInd1[i] = closestPointInd;
              pointSearchCornerInd2[i] = minPointInd2;
            }

            if (pointSearchCornerInd2[i] >= 0) {
              tripod1 = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
              tripod2 = laserCloudCornerLast->points[pointSearchCornerInd2[i]];

              float x0 = pointSel.x;
              float y0 = pointSel.y;
              float z0 = pointSel.z;
              float x1 = tripod1.x;
              float y1 = tripod1.y;
              float z1 = tripod1.z;
              float x2 = tripod2.x;
              float y2 = tripod2.y;
              float z2 = tripod2.z;

              float a012 =
                  sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) *
                           ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
                       ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) *
                           ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                       ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) *
                           ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

              float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) +
                               (z1 - z2) * (z1 - z2));

              float la =
                  ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
                   (z1 - z2) *
                       ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) /
                  a012 / l12;

              float lb = -((x1 - x2) *
                               ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) -
                           (z1 - z2) * ((y0 - y1) * (z0 - z2) -
                                        (y0 - y2) * (z0 - z1))) /
                         a012 / l12;

              float lc = -((x1 - x2) *
                               ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                           (y1 - y2) * ((y0 - y1) * (z0 - z2) -
                                        (y0 - y2) * (z0 - z1))) /
                         a012 / l12;

              float ld2 = a012 / l12;

              pointProj = pointSel;
              pointProj.x -= la * ld2;
              pointProj.y -= lb * ld2;
              pointProj.z -= lc * ld2;

              float s = 1;
              if (iterCount >= 5) {
                s = 1 - 1.8 * fabs(ld2);
              }

              coeff.x = s * la;
              coeff.y = s * lb;
              coeff.z = s * lc;
              coeff.intensity = s * ld2;

              if (s > 0.1 && ld2 != 0) {
                laserCloudOri->push_back(CurrentCornerFrame->points[i]);
                coeffSel->push_back(coeff);
              }
            }
          }
          // cout<<"after corner iterate"<<endl;
          for (int i = 0; i < CurrentSurfNum; i++) {
            TransformToStart(&CurrentSurfinWorld->points[i], &pointSel);
            if (iterCount % 5 == 0) {
              // cout<<"surffrommap kn search"<<endl;
              int ssresults2 = kdtreeSurfFromMap->nearestKSearch(pointSel, 1, pointSearchInd,
                                                pointSearchSqDis);
              int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
              if (pointSearchSqDis[0] < 25) {
                closestPointInd = pointSearchInd[0];
                int closestPointScan =
                    int(laserCloudSurfLast->points[closestPointInd].intensity);

                float pointSqDis, minPointSqDis2 = 25, minPointSqDis3 = 25;
                for (int j = closestPointInd + 1; j < CurrentSurfNum; j++) {
                  if (int(laserCloudSurfLast->points[j].intensity) >
                      closestPointScan + 2.5) {
                    break;
                  }

                  pointSqDis =
                      (laserCloudSurfLast->points[j].x - pointSel.x) *
                          (laserCloudSurfLast->points[j].x - pointSel.x) +
                      (laserCloudSurfLast->points[j].y - pointSel.y) *
                          (laserCloudSurfLast->points[j].y - pointSel.y) +
                      (laserCloudSurfLast->points[j].z - pointSel.z) *
                          (laserCloudSurfLast->points[j].z - pointSel.z);

                  if (int(laserCloudSurfLast->points[j].intensity) <=
                      closestPointScan) {
                    if (pointSqDis < minPointSqDis2) {
                      minPointSqDis2 = pointSqDis;
                      minPointInd2 = j;
                    }
                  } else {
                    if (pointSqDis < minPointSqDis3) {
                      minPointSqDis3 = pointSqDis;
                      minPointInd3 = j;
                    }
                  }
                }
                for (int j = closestPointInd - 1; j >= 0; j--) {
                  if (int(laserCloudSurfLast->points[j].intensity) <
                      closestPointScan - 2.5) {
                    break;
                  }

                  pointSqDis =
                      (laserCloudSurfLast->points[j].x - pointSel.x) *
                          (laserCloudSurfLast->points[j].x - pointSel.x) +
                      (laserCloudSurfLast->points[j].y - pointSel.y) *
                          (laserCloudSurfLast->points[j].y - pointSel.y) +
                      (laserCloudSurfLast->points[j].z - pointSel.z) *
                          (laserCloudSurfLast->points[j].z - pointSel.z);

                  if (int(laserCloudSurfLast->points[j].intensity) >=
                      closestPointScan) {
                    if (pointSqDis < minPointSqDis2) {
                      minPointSqDis2 = pointSqDis;
                      minPointInd2 = j;
                    }
                  } else {
                    if (pointSqDis < minPointSqDis3) {
                      minPointSqDis3 = pointSqDis;
                      minPointInd3 = j;
                    }
                  }
                }
              }

              pointSearchSurfInd1[i] = closestPointInd;
              pointSearchSurfInd2[i] = minPointInd2;
              pointSearchSurfInd3[i] = minPointInd3;
            }

            if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0) {
              tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]];
              tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]];
              tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]];

              float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z) -
                         (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
              float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x) -
                         (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
              float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y) -
                         (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
              float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

              float ps = sqrt(pa * pa + pb * pb + pc * pc);
              pa /= ps;
              pb /= ps;
              pc /= ps;
              pd /= ps;

              float pd2 =
                  pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

              pointProj = pointSel;
              pointProj.x -= pa * pd2;
              pointProj.y -= pb * pd2;
              pointProj.z -= pc * pd2;

              float s = 1;
              if (iterCount >= 5) {
                s = 1 -
                    1.8 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x +
                                                pointSel.y * pointSel.y +
                                                pointSel.z * pointSel.z));
              }

              coeff.x = s * pa;
              coeff.y = s * pb;
              coeff.z = s * pc;
              coeff.intensity = s * pd2;

              if (s > 0.1 && pd2 != 0) {
                laserCloudOri->push_back(CurrentSurfFrame->points[i]);
                coeffSel->push_back(coeff);
              }
            }
          }
          // cout<<"after surf iterate"<<endl;
          int pointSelNum = laserCloudOri->points.size();
          if (pointSelNum < 10) {
            continue;
          }

          cv::Mat matA(pointSelNum, 6, CV_32F, cv::Scalar::all(0));
          cv::Mat matAt(6, pointSelNum, CV_32F, cv::Scalar::all(0));
          cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
          cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
          cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
          cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
          for (int i = 0; i < pointSelNum; i++) {
            pointOri = laserCloudOri->points[i];
            coeff = coeffSel->points[i];

            float srx = sin(Robottransform[0]);
            float crx = cos(Robottransform[0]);
            float sry = sin(Robottransform[1]);
            float cry = cos(Robottransform[1]);
            float srz = sin(Robottransform[2]);
            float crz = cos(Robottransform[2]);
            float tx = Robottransform[3];
            float ty = Robottransform[4];
            float tz = Robottransform[5];

            float arx =
                (-crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y +
                 srx * sry * pointOri.z + tx * crx * sry * srz -
                 ty * crx * crz * sry - tz * srx * sry) *
                    coeff.x +
                (srx * srz * pointOri.x - crz * srx * pointOri.y +
                 crx * pointOri.z + ty * crz * srx - tz * crx -
                 tx * srx * srz) *
                    coeff.y +
                (crx * cry * srz * pointOri.x - crx * cry * crz * pointOri.y -
                 cry * srx * pointOri.z + tz * cry * srx +
                 ty * crx * cry * crz - tx * crx * cry * srz) *
                    coeff.z;

            float ary =
                ((-crz * sry - cry * srx * srz) * pointOri.x +
                 (cry * crz * srx - sry * srz) * pointOri.y -
                 crx * cry * pointOri.z + tx * (crz * sry + cry * srx * srz) +
                 ty * (sry * srz - cry * crz * srx) + tz * crx * cry) *
                    coeff.x +
                ((cry * crz - srx * sry * srz) * pointOri.x +
                 (cry * srz + crz * srx * sry) * pointOri.y -
                 crx * sry * pointOri.z + tz * crx * sry -
                 ty * (cry * srz + crz * srx * sry) -
                 tx * (cry * crz - srx * sry * srz)) *
                    coeff.z;

            float arz = ((-cry * srz - crz * srx * sry) * pointOri.x +
                         (cry * crz - srx * sry * srz) * pointOri.y +
                         tx * (cry * srz + crz * srx * sry) -
                         ty * (cry * crz - srx * sry * srz)) *
                            coeff.x +
                        (-crx * crz * pointOri.x - crx * srz * pointOri.y +
                         ty * crx * srz + tx * crx * crz) *
                            coeff.y +
                        ((cry * crz * srx - sry * srz) * pointOri.x +
                         (crz * sry + cry * srx * srz) * pointOri.y +
                         tx * (sry * srz - cry * crz * srx) -
                         ty * (crz * sry + cry * srx * srz)) *
                            coeff.z;

            float atx = -(cry * crz - srx * sry * srz) * coeff.x +
                        crx * srz * coeff.y -
                        (crz * sry + cry * srx * srz) * coeff.z;

            float aty = -(cry * srz + crz * srx * sry) * coeff.x -
                        crx * crz * coeff.y -
                        (sry * srz - cry * crz * srx) * coeff.z;

            float atz =
                crx * sry * coeff.x - srx * coeff.y - crx * cry * coeff.z;

            float d2 = coeff.intensity;

            matA.at<float>(i, 0) = arx;
            matA.at<float>(i, 1) = ary;
            matA.at<float>(i, 2) = arz;
            matA.at<float>(i, 3) = atx;
            matA.at<float>(i, 4) = aty;
            matA.at<float>(i, 5) = atz;
            matB.at<float>(i, 0) = -0.05 * d2;
          }
          cv::transpose(matA, matAt);
          matAtA = matAt * matA;
          matAtB = matAt * matB;
          cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

          if (iterCount == 0) {
            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {10, 10, 10, 10, 10, 10};
            for (int i = 5; i >= 0; i--) {
              if (matE.at<float>(0, i) < eignThre[i]) {
                for (int j = 0; j < 6; j++) {
                  matV2.at<float>(i, j) = 0;
                }
                isDegenerate = true;
              } else {
                break;
              }
            }
            matP = matV.inv() * matV2;
          }

          if (isDegenerate) {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
          }

          Robottransform[0] += matX.at<float>(0, 0);
          Robottransform[1] += matX.at<float>(1, 0);
          Robottransform[2] += matX.at<float>(2, 0);
          Robottransform[3] += matX.at<float>(3, 0);
          Robottransform[4] += matX.at<float>(4, 0);
          Robottransform[5] += matX.at<float>(5, 0);

          /*
           * ScanToMap[0] -= matX.at<float>(0, 0);
           * ScanToMap[1] -= matX.at<float>(1, 0);
           * ScanToMap[2] -= matX.at<float>(2, 0);
           * ScanToMap[3] -= matX.at<float>(3, 0);
           * ScanToMap[4] -= matX.at<float>(4, 0);
           * ScanToMap[5] -= matX.at<float>(5, 0);
           */
          for (int i = 0; i < 6; i++) {
            if (isnan(ScanToMap[i])) ScanToMap[i] = 0;
          }
          float deltaR = sqrt(pow(rad2deg(matX.at<float>(0, 0)), 2) +
                              pow(rad2deg(matX.at<float>(1, 0)), 2) +
                              pow(rad2deg(matX.at<float>(2, 0)), 2));
          float deltaT = sqrt(pow(matX.at<float>(3, 0) * 100, 2) +
                              pow(matX.at<float>(4, 0) * 100, 2) +
                              pow(matX.at<float>(5, 0) * 100, 2));

          if (deltaR < 0.1 && deltaT < 0.1) {
            std::cout << "iterCount = " << iterCount << " OK " << endl;
            break;
          }
        }
      }
    }
    status = ros::ok();
    rate.sleep();
  }

  return (0);
}