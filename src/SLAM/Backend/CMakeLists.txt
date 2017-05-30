cmake_minimum_required(VERSION 2.6.0)
project(libSLAM)

set(SLAM_HEADERS
NuiSLAMController.h
NuiSLAMPointCloud.h
NuiSLAMPointCloudManager.h
)

set(SLAM_SOURCES
NuiSLAMController.cpp
NuiSLAMPointCloud.cpp
NuiSLAMPointCloudManager.cpp
)

SOURCE_GROUP("" FILES ${SLAM_HEADERS} ${SLAM_SOURCES})

add_definitions(-DNOMINMAX)
add_definitions(-DWITH_OPENMP)

include_directories($ENV{NUI_LOCATION})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${FLANN_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${log4cplus_DIR}/../../../include)
include_directories($ENV{MAYA_RELEASE_BUILD}/include)

add_library(libSLAM
			${SLAM_HEADERS}
			${SLAM_SOURCES}
)

target_link_libraries(libSLAM
					libVisualOdometry
					${PCL_FILTERS_LIBRARIES}
					${PCL_FEATURES_LIBRARIES}
)