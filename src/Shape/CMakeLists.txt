cmake_minimum_required(VERSION 2.6.0)
project(libShape)

set(SHAPE_HEADERS
NuiCameraParams.h
NuiCameraPos.h
NuiCLMappableData.h
NuiCompoundImage.h
NuiDevicePointCloud.h
NuiFacialModel.h
NuiFusionMesh.h
NuiGestureResponse.h
NuiGestureResult.h
NuiImageBuffer.h
NuiMemoryBlock.h
NuiMeshShape.h
NuiPointCloudImpl.h
NuiPointCloudShape.h
NuiSkeletonJoints.h
NuiTrackedFace.h
)

set(SHAPE_SOURCES
NuiCLMappableData.cpp
NuiCompoundImage.cpp
NuiDevicePointCloud.cpp
NuiFacialModel.cpp
NuiGestureResponse.cpp
NuiGestureResult.cpp
NuiSkeletonJoints.cpp
NuiTrackedFace.cpp
)

include_directories($ENV{NUI_LOCATION})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})

add_library(libShape
			${SHAPE_HEADERS}
			${SHAPE_SOURCES}
)
target_link_libraries(libShape ${Boost_LIBRARIES})