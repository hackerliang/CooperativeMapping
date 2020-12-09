#include <iostream>
#include <iomanip>
#include <ctime>
#include <loam_velodyne/common.h>
#include <boost/format.hpp>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <laser_slam/parameters.hpp>
#include <laser_slam/incremental_estimator.hpp>
#include <laser_slam_ros/laser_slam_worker.hpp>
#include <segmatch/common.hpp>
#include <segmatch/local_map.hpp>
#include <segmatch_ros/common.hpp>
#include <segmatch_ros/segmatch_worker.hpp>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <laser_slam/benchmarker.hpp>
#include <laser_slam/common.hpp>
#include <laser_slam_ros/common.hpp>
#include <segmatch/utilities.hpp>
#include <tf_conversions/tf_eigen.h>
#include <pointmatcher_ros/point_cloud.h>
#include <pointmatcher_ros/transform.h>
#include <pointmatcher/PointMatcher.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include "DataCollect.h"
#include "../include/loam_velodyne_msg/Heading.h"
#include "../include/loam_velodyne_msg/NodeTransform.h"

using namespace std;
using namespace laser_slam;
using namespace laser_slam_ros;
using namespace segmatch;
using namespace segmatch_ros;
using namespace message_filters;

pcl::PointCloud<pcl::PointXYZ> thisCarMapForLP;
pcl::PointCloud<pcl::PointXYZ> otherCarMapForLP;

ofstream recordtime0("/home/xyz/segmap/data/thismatchingtimes.txt", std::ios::trunc);
ofstream recordtime1("/home/xyz/segmap/data/othermatchingtimes.txt", std::ios::trunc);

std::vector<segmatch::LocalMap<pcl::PointXYZ, segmatch::MapPoint>> local_maps_;
std::recursive_mutex scan_callback_mutex_;
std::recursive_mutex full_laser_track_mutex_;

// Parameters.
SegMapperParams params_;
laser_slam::BenchmarkerParams benchmarker_params_;
pcl::PointCloud<pcl::PointXYZ> output;

// Incremental estimator.
std::shared_ptr<laser_slam::IncrementalEstimator> incremental_estimator_;
Pose thispose,otherpose,last_pose;

// SegMatch objects.
segmatch_ros::SegMatchWorkerParams segmatch_worker_params_;
segmatch_ros::SegMatchWorker segmatch_worker_;
static constexpr double kSegMatchSleepTime_s = 0.01;

// laser_slam objects.
std::vector<std::unique_ptr<laser_slam_ros::LaserSlamWorker> > laser_slam_workers_;
laser_slam_ros::LaserSlamWorkerParams laser_slam_worker_params_;

std::vector<unsigned int> skip_counters_;
unsigned int deactivate_track_when_skipped_x_ = 5u;
std::vector<bool> first_points_received_;
std::vector<pcl::PointCloud<pcl::PointXYZ>> new_points_queue;
std::vector<pcl::PointCloud<pcl::PointXYZ>> other_points_queue;

bool lock_scan_callback_ = false;
bool lock_scan_callback1_ = false;
bool ifGetMapThisCar = false, ifGetCloudOthers = false, notalreadysave = true;
double dist = 0;

void print4x4Matrix(const Eigen::Matrix4f & matrix) {
	std::cout << setw(15) << matrix(0,0) << setw(15) << matrix(0,1) << setw(15) << matrix(0,2) << setw(15) << matrix(0,3) << std::endl;
	std::cout << setw(15) << matrix(1,0) << setw(15) << matrix(1,1) << setw(15) << matrix(1,2) << setw(15) << matrix(1,3) << std::endl;
	std::cout << setw(15) << matrix(2,0) << setw(15) << matrix(2,1) << setw(15) << matrix(2,2) << setw(15) << matrix(2,3) << std::endl;
	std::cout << setw(15) << matrix(3,0) << setw(15) << matrix(3,1) << setw(15) << matrix(3,2) << setw(15) << matrix(3,3) << std::endl;
}


void setLockScanCallback(bool new_state) {
  std::lock_guard<std::recursive_mutex> lock(scan_callback_mutex_);
  lock_scan_callback_ = new_state;
}


void processthisposecandscan(const nav_msgs::Odometry::ConstPtr& pose,const sensor_msgs::PointCloud2ConstPtr& laserCloudPoint) {  
  std::lock_guard<std::recursive_mutex> lock_scan_callback(scan_callback_mutex_);
  if(!lock_scan_callback_){
    gtsam::NonlinearFactorGraph new_factors0;
    gtsam::Values new_values0;
    bool is_prior0;
    thisCarMapForLP.clear();
    // bool process_scan = false;
    // SE3 current_pose;
    // if (!last_pose_set_) {
    //   process_scan = true;
    //   last_pose_set_ = true;
    //   last_pose = laser_slam_workers_[0u]->tfTransformToPose(tf_transform).T_w;
    // } else {
    //   current_pose = laser_slam_workers_[0u]->tfTransformToPose(tf_transform).T_w;
    //   float dist_m = distanceBetweenTwoSE3(current_pose, last_pose);
    //   if (dist_m > params_.minimum_distance_add_pose) {
    //     process_scan = true;
    //     last_pose = current_pose;
    //   }
    // }
    pcl::fromROSMsg(*laserCloudPoint, thisCarMapForLP);

    LaserScan thisscan;  
    thisscan.time_ns = laser_slam_workers_[0]->rosTimeToCurveTime(laserCloudPoint->header.stamp.toNSec());

    thispose.T_w=laser_slam_workers_[0]->navOdometryToPose(pose).T_w;
    thispose.time_ns=laser_slam_workers_[0]->navOdometryToPose(pose).time_ns;

    laser_slam_workers_[0]->laser_track_->processPoseAndLaserScan(thispose, thisscan,
                                        &new_factors0, &new_values0, &is_prior0);

    // recordtime0<<setiosflags(ios::scientific)<<laserCloudPoint->header.stamp.toSec()<<std::endl;

    gtsam::Values result0;
    if (is_prior0) {
      result0 = incremental_estimator_->registerPrior(new_factors0, new_values0, 0);
    } 
    else {
      result0 = incremental_estimator_->estimate(new_factors0, new_values0, thisscan.time_ns);
    }
   
    laser_slam_workers_[0]->laser_track_->updateFromGTSAMValues(result0);

    {
      std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
      Pose new_pose = laser_slam_workers_[0u]->laser_track_->getCurrentPose();
      laser_slam::SE3 transform_matrix = new_pose.T_w * thispose.T_w.inverse();
      pcl::transformPointCloud (thisCarMapForLP,output,transform_matrix.getTransformationMatrix().cast<float>());
      new_points_queue.push_back(output);
      // std::cout<<"new_points_queue "<<new_points_queue.size()<<std::endl;
      if(!output.empty()) {
        ifGetMapThisCar = true;
      }
    }
  }
}

void processotherposecandscan(const nav_msgs::Odometry::ConstPtr& pose,const sensor_msgs::PointCloud2ConstPtr& laserCloudPoint)
{
  std::lock_guard<std::recursive_mutex> lock_scan_callback(scan_callback_mutex_);
  if(!lock_scan_callback1_){
    gtsam::NonlinearFactorGraph new_factors1;
    gtsam::Values new_values1;
    bool is_prior1;
    otherCarMapForLP.clear();
    pcl::fromROSMsg(*laserCloudPoint, otherCarMapForLP);
   
    LaserScan otherscan;  
    otherscan.time_ns = laser_slam_workers_[1]->rosTimeToCurveTime(laserCloudPoint->header.stamp.toNSec());

    otherpose.T_w=laser_slam_workers_[1]->navOdometryToPose(pose).T_w;
    otherpose.time_ns=laser_slam_workers_[1]->navOdometryToPose(pose).time_ns;
 
    laser_slam_workers_[1]->laser_track_->processPoseAndLaserScan(otherpose, otherscan,
                                        &new_factors1, &new_values1, &is_prior1);
    // recordtime1<<setiosflags(ios::scientific)<<laserCloudPoint->header.stamp.toSec()<<std::endl;
   
    gtsam::Values result1;
    if (is_prior1) {
      result1 = incremental_estimator_->registerPrior(new_factors1, new_values1, 1);
    } 
    else {
      result1 = incremental_estimator_->estimate(new_factors1, new_values1, otherscan.time_ns);
    }
   
    laser_slam_workers_[1]->laser_track_->updateFromGTSAMValues(result1);
   
    {
      std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
      Pose new_pose = laser_slam_workers_[1u]->laser_track_->getCurrentPose();
      laser_slam::SE3 transform_matrix = new_pose.T_w * otherpose.T_w.inverse();
      pcl::transformPointCloud (otherCarMapForLP,output,transform_matrix.getTransformationMatrix().cast<float>());
      other_points_queue.push_back(output);
      // std::cout<<"other_points_queue "<<other_points_queue.size()<<std::endl;
      if(!output.empty()) {
        ifGetCloudOthers = true;
      }
    }
  }
}


void getParameters(ros::NodeHandle& nh_) {
  // SegMapper parameters.
	const std::string ns = "/SegMapper";
	nh_.getParam(ns + "/number_of_robots",
	           params_.number_of_robots);
	nh_.getParam(ns + "/robot_prefix",
	           params_.robot_prefix);

	CHECK_GE(params_.number_of_robots, 0u);

	nh_.getParam(ns + "/publish_world_to_odom",
	           params_.publish_world_to_odom);
	nh_.getParam(ns + "/world_frame",
	           params_.world_frame);
	nh_.getParam(ns + "/tf_publication_rate_hz",
	           params_.tf_publication_rate_hz);

	nh_.getParam(ns + "/clear_local_map_after_loop_closure",
	           params_.clear_local_map_after_loop_closure);

	// laser_slam worker parameters.
	laser_slam_worker_params_ = laser_slam_ros::getLaserSlamWorkerParams(nh_, ns);
	laser_slam_worker_params_.world_frame = params_.world_frame;

	// Online estimator parameters.
	params_.online_estimator_params = laser_slam_ros::getOnlineEstimatorParams(nh_, ns);

	// Benchmarker parameters.
	benchmarker_params_ = laser_slam_ros::getBenchmarkerParams(nh_, ns);

	// ICP configuration files.
	nh_.getParam("icp_configuration_file",
	           params_.online_estimator_params.laser_track_params.icp_configuration_file);
	nh_.getParam("icp_input_filters_file",
	           params_.online_estimator_params.laser_track_params.icp_input_filters_file);

	// SegMatchWorker parameters.
	segmatch_worker_params_ = segmatch_ros::getSegMatchWorkerParams(nh_, ns);
	segmatch_worker_params_.world_frame = params_.world_frame;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "matching");
  ros::NodeHandle nh;
  getParameters(nh);
  
  unsigned int track_id = laser_slam_workers_.size() - 1u;
  unsigned int skipped_tracks_count = 0u;
  ros::Duration sleep_duration(kSegMatchSleepTime_s);
  const std::string& segmenter_type = segmatch_worker_params_.segmatch_params.segmenter_params.segmenter_type;
  const bool needs_normal_estimation = (segmenter_type == "SimpleSmoothnessConstraints") ||
    (segmenter_type == "IncrementalSmoothnessConstraints");

  std::shared_ptr<IncrementalEstimator> incremental_estimator(new IncrementalEstimator(params_.online_estimator_params, params_.number_of_robots));
  incremental_estimator_ = incremental_estimator;

  // Setup the laser_slam workers.
  ROS_INFO_STREAM("Number of laser_slam workers: " << params_.number_of_robots);
  for (unsigned int i = 0u; i < params_.number_of_robots; ++i) {
    // Adjust the topics and frames for that laser_slam worker.
    LaserSlamWorkerParams params = laser_slam_worker_params_;

    // Create a local map for each robot.
    std::unique_ptr<NormalEstimator> normal_estimator = nullptr;
    if (needs_normal_estimation) {
      normal_estimator = NormalEstimator::create(
      segmatch_worker_params_.segmatch_params.normal_estimator_type,
      segmatch_worker_params_.segmatch_params.radius_for_normal_estimation_m);
    }
    local_maps_.emplace_back(segmatch_worker_params_.segmatch_params.local_map_params, std::move(normal_estimator));
  
    // TODO rm offset when updating mr_foundry.
    const unsigned int offset = 0;
    if (params_.number_of_robots > 1) {
      params.assembled_cloud_sub_topic = "/" + params_.robot_prefix + std::to_string(i + offset) +
        "/" + laser_slam_worker_params_.assembled_cloud_sub_topic;

      params.odom_frame =  params_.robot_prefix + std::to_string(i + offset) +
        "/" + laser_slam_worker_params_.odom_frame;
      params.sensor_frame =  params_.robot_prefix + std::to_string(i + offset) +
        "/" + laser_slam_worker_params_.sensor_frame;
        
       params.trajectory_pub_topic = params_.robot_prefix + std::to_string(i + offset) + "/" +
      laser_slam_worker_params_.trajectory_pub_topic;
   
      params.local_map_pub_topic = params_.robot_prefix + std::to_string(i + offset) + "/" +
        laser_slam_worker_params_.local_map_pub_topic;
    }

    LOG(INFO) << "Robot " << i << " subscribes to " << params.assembled_cloud_sub_topic << " "
        << params.odom_frame << " and " << params.sensor_frame;

    LOG(INFO) << "Robot " << i << " publishes to " << params.trajectory_pub_topic << " and "
        << params.local_map_pub_topic;

    std::unique_ptr<LaserSlamWorker> laser_slam_worker(new LaserSlamWorker());
    laser_slam_worker->init(nh, params, incremental_estimator_, i);
    laser_slam_workers_.push_back(std::move(laser_slam_worker));
  }

  if (segmatch_worker_params_.close_loops) {
    segmatch_worker_.init(nh, segmatch_worker_params_, params_.number_of_robots);
  }

  for (size_t i = 0u; i < laser_slam_workers_.size(); ++i) {
    skip_counters_.push_back(0u);
    first_points_received_.push_back(false);
  }

	
  message_filters::Subscriber<nav_msgs::Odometry> subthisPose(nh,"/aft_mapped_to_init", 2);
  message_filters::Subscriber<nav_msgs::Odometry> subotherPose(nh,"/pose_from_others", 2);
  message_filters::Subscriber<sensor_msgs::PointCloud2> subthiscan(nh,"/velodyne_cloud_registered", 2);
  message_filters::Subscriber<sensor_msgs::PointCloud2> subotherscan(nh,"/map_for_rt_from_others", 2);
  TimeSynchronizer<nav_msgs::Odometry,sensor_msgs::PointCloud2> sync(subthisPose,subthiscan,2);
  sync.registerCallback(boost::bind(&processthisposecandscan,_1,_2));
  TimeSynchronizer<nav_msgs::Odometry,sensor_msgs::PointCloud2> sync2(subotherPose,subotherscan,2);
  sync2.registerCallback(boost::bind(&processotherposecandscan,_1,_2));

  PointType thistrackcloudpart,othertrackcloudpart;
	// ofstream traject0("/home/xyz/segmap/data/trajectory/path0.txt", ios::trunc);
 //  ofstream traject1("/home/xyz/segmap/data/trajectory/path1.txt", ios::trunc);
  std::ofstream record("/home/xyz/segmap/data/loop.txt", std::ios::trunc);

	ros::Rate rate(10);
	bool status = ros::ok();
	double score = 0;
	//clock_t start, end;
  
	while(status) {
    // while(ifGetMapThisCar == true && ifGetCloudOthers == true){
    while(ifGetMapThisCar == true || ifGetCloudOthers == true){
      ros::spinOnce();
      rate.sleep();
      // std::cout<<"start"<<std::endl;
      if (skipped_tracks_count == laser_slam_workers_.size()) {
        skipped_tracks_count = 0u;
        sleep_duration.sleep();
      }  
      BENCHMARK_START_NEW_STEP();
      BENCHMARK_START("SM");
      track_id = (track_id + 1u) % laser_slam_workers_.size();
      // std::cout<<"track_id: "<<track_id<<std::endl;

      std::vector<laser_slam_ros::PointCloud> new_points;
      if(track_id == 0) { 
        new_points.swap(new_points_queue);  
      }  
      else {   
        new_points.swap(other_points_queue); 
      }
       // std::cout<<"track id "<<track_id<<" size "<<new_points.size()<<std::endl;    
      if (new_points.size() < 1) {
        BENCHMARK_STOP_AND_IGNORE("SM");
        ++skipped_tracks_count;
        segmatch_worker_.publish();
        skip_counters_[track_id]++;

        if (first_points_received_[track_id] && skip_counters_[track_id] == deactivate_track_when_skipped_x_) {
          // std::cout<<"skip_counters_: "<<skip_counters_[track_id]<<" "<<deactivate_track_when_skipped_x_<<std::endl;
          bool is_one_still_active = false;
          for (const auto& counter : skip_counters_) {
            if (counter <= 1u) is_one_still_active = true;
          }
          if (is_one_still_active) {
            segmatch_worker_.stopPublishing(track_id);
          } else {
            skip_counters_[track_id] = 0u;
          }
        }
        continue;
      } else {
            // std::cout<<"getCurrentPose"<<std::endl;
            // new_points.swap(new_points_queue);
            // std::cout<<"queue "<<new_points.size()<<std::endl;
        if (!first_points_received_[track_id]) {
          first_points_received_[track_id] = true;
          skip_counters_[track_id] = 0u;
        }
      }
    
      Pose localmap_pose = incremental_estimator_->getCurrentPose(track_id);
      local_maps_[track_id].updatePoseAndAddPoints(new_points, localmap_pose);
     
      RelativePose loop_closure;
    
      // If there is a loop closure.
      if (segmatch_worker_.processLocalMap(local_maps_[track_id], localmap_pose,
                                           track_id, &loop_closure)) {
        BENCHMARK_BLOCK("SM.ProcessLoopClosure");
       
        Eigen::Matrix4f transformation_ICP = loop_closure.T_a_b.getTransformationMatrix().cast<float>();
        std::cout<< "Found loop closure! track_id_a: " << loop_closure.track_id_a <<
            " time_a_ns: " << loop_closure.time_a_ns <<
            " track_id_b: " << loop_closure.track_id_b <<
            " time_b_ns: " << loop_closure.time_b_ns<<std::endl;
        std::cout<<transformation_ICP<<std::endl;
        // record<<transformation_ICP<<std::endl;
        // print4x4Matrix(transformation_ICP); 

        // Prevent the workers to process further scans (and add variables to the graph).
        BENCHMARK_START("SM.ProcessLoopClosure.WaitingForLockOnLaserSlamWorkers");
        setLockScanCallback(true); 
        BENCHMARK_STOP("SM.ProcessLoopClosure.WaitingForLockOnLaserSlamWorkers");

        BENCHMARK_START("SM.ProcessLoopClosure.UpdateIncrementalEstimator");
        incremental_estimator_->processLoopClosure(loop_closure);
        BENCHMARK_STOP("SM.ProcessLoopClosure.UpdateIncrementalEstimator");

        // Update the Segmatch object.
        Trajectory trajectory;
        std::vector<Trajectory> updated_trajectories;
        for (const auto& worker: laser_slam_workers_) {
          worker->getTrajectory(&trajectory);
          updated_trajectories.push_back(trajectory);
        }
       
        BENCHMARK_START("SM.ProcessLoopClosure.UpdateSegMatch");
        segmatch_worker_.update(updated_trajectories);
        BENCHMARK_STOP("SM.ProcessLoopClosure.UpdateSegMatch");
      
        //Publish the trajectories.
        for (const auto& worker : laser_slam_workers_) {
          worker->publishTrajectories();
        }
   
        setLockScanCallback(false); 
        // n_loops++;
        // LOG(INFO) << "That was the loop number " << n_loops << ".";

    }//localmap
    for (const auto& worker : laser_slam_workers_) {
      worker->publishTrajectories();
    }
  
    skipped_tracks_count = 0;
    skip_counters_[track_id] = 0u;
    BENCHMARK_STOP("SM");
    // std::cout<<"end"<<std::endl;

    // if(track_id == 0) {
    //   dist += distanceBetweenTwoSE3(last_pose.T_w, localmap_pose.T_w);
    //   last_pose = localmap_pose;
    //   std::cout<<"dist "<<dist<<std::endl;
    // }
    // if(dist >= 1328) {
    //   if(notalreadysave) {
    //     // std::cout<<"start save data"<<std::endl;
    //     Trajectory trajectory0;
    //     laser_slam_workers_[0]->getTrajectory(&trajectory0);

    //     for (const auto& timePose : trajectory0) {
    //       // traject0<<timePose.second.getPosition().x()<<' '<<timePose.second.getPosition().y()<<' '<<timePose.second.getPosition().z()<<std::endl;
    //       Eigen::Matrix4f eigentrac0 = timePose.second.getTransformationMatrix().cast<float>();
    //       traject0<<setiosflags(ios::scientific)<<eigentrac0(0,0)<<' '<<eigentrac0(0,1)<<' '<<eigentrac0(0,2)<<' '<<eigentrac0(0,3)<<' '<<
    //                 eigentrac0(1,0)<<' '<<eigentrac0(1,1)<<' '<<eigentrac0(1,2)<<' '<<eigentrac0(1,3)<<' '<<
    //                 eigentrac0(2,0)<<' '<<eigentrac0(2,1)<<' '<<eigentrac0(2,2)<<' '<<eigentrac0(2,3)<<std::endl;
    //     }

    //     Trajectory trajectory1;
    //     laser_slam_workers_[1]->getTrajectory(&trajectory1);

    //     for (const auto& timePose : trajectory1) {
    //       // traject1<<timePose.second.getPosition().x()<<' '<<timePose.second.getPosition().y()<<' '<<timePose.second.getPosition().z()<<std::endl;
    //       Eigen::Matrix4f eigentrac1 = timePose.second.getTransformationMatrix().cast<float>();
    //       traject1<<setiosflags(ios::scientific)<<eigentrac1(0,0)<<' '<<eigentrac1(0,1)<<' '<<eigentrac1(0,2)<<' '<<eigentrac1(0,3)<<' '<<
    //                 eigentrac1(1,0)<<' '<<eigentrac1(1,1)<<' '<<eigentrac1(1,2)<<' '<<eigentrac1(1,3)<<' '<<
    //                 eigentrac1(2,0)<<' '<<eigentrac1(2,1)<<' '<<eigentrac1(2,2)<<' '<<eigentrac1(2,3)<<std::endl;
    //     }

    //     std::cout<<"complete"<<std::endl;
    //     notalreadysave = false;
    //   }
    // }

  }//IF

		ros::spinOnce();
		status = ros::ok();
    rate.sleep();
  
	}//status
 
	return 0;
}//main


