cmake_minimum_required(VERSION 2.8.3)
project(loam_velodyne)
find_library(LCM lcm)
add_definitions(-std=c++11 -fext-numeric-literals -DBENCHMARK_ENABLE)


find_package(catkin_simple 0.1.0 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(Ceres REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})


add_executable(scanRegistration src/scanRegistration.cpp)
target_link_libraries(scanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(RobotLocation src/RobotLocation.cpp)
target_link_libraries(RobotLocation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(laserOdometry src/laserOdometry.cpp)
target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(laserMapping src/laserMapping.cpp)
target_link_libraries(laserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OCTOMAP_LIBRARIES} ${CERES_LIBRARIES})

add_executable(transformMaintenance src/transformMaintenance.cpp)
target_link_libraries(transformMaintenance ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(Communicate src/Communicate.cpp src/DataCollect.h src/DataCollect.cpp)
target_link_libraries(Communicate ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${LCM} -lpthread )

add_executable(Matching src/Matching.cpp src/DataCollect.h src/DataCollect.cpp)
target_link_libraries(Matching ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} -lpthread)


cs_install()
cs_export()






