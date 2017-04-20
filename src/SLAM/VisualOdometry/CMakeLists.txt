cmake_minimum_required(VERSION 2.6.0)
project(libVisualOdometry)

set(VO_HEADERS
NuiHashingChunkGridConfig.h
NuiHashingRaycastConfig.h
NuiHashingSDFConfig.h
NuiKinfuMainEngine.h
NuiKinfuManager.h
NuiKinfuPointCloud.h
NuiKinfuTrackingEngine.h
NuiKinfuVolumeConfig.h
NuiMeshingUtil.h
NuiTrackerConfig.h
)

set(VO_SOURCES
NuiHashingChunkGridConfig.cpp
NuiHashingRaycastConfig.cpp
NuiHashingSDFConfig.cpp
NuiKinfuMainEngine.cpp
NuiKinfuManager.cpp
NuiKinfuTrackingEngine.cpp
NuiKinfuVolumeConfig.cpp
NuiMeshingUtil.cpp
NuiTrackerConfig.cpp
)

set(VO_DEVICE_HEADERS
DeviceSpecific/NuiKinfuCameraState.h
DeviceSpecific/NuiKinfuFeedbackFrame.h
DeviceSpecific/NuiKinfuFrame.h
DeviceSpecific/NuiKinfuScene.h
DeviceSpecific/NuiKinfuTracker.h
DeviceSpecific/NuiKinfuTrackingFactory.h
)

set(VO_DEVICE_CPU_HEADERS
DeviceSpecific/CPU/NuiKinfuCPUDepthTracker.h
DeviceSpecific/CPU/NuiKinfuCPUFeedbackFrame.h
DeviceSpecific/CPU/NuiKinfuCPUFrame.h
DeviceSpecific/CPU/NuiKinfuCPUUtilities.h
)

set(VO_DEVICE_CPU_SOURCES
DeviceSpecific/CPU/NuiKinfuCPUDepthTracker.cpp
DeviceSpecific/CPU/NuiKinfuCPUFeedbackFrame.cpp
DeviceSpecific/CPU/NuiKinfuCPUFrame.cpp
)

set(VO_DEVICE_OPENCL_HEADERS
DeviceSpecific/OpenCL/NuiHashingOpenCLChunkGrid
DeviceSpecific/OpenCL/NuiHashingOpenCLScene
DeviceSpecific/OpenCL/NuiHashingOpenCLSDF
DeviceSpecific/OpenCL/NuiKinfuOpenCLAcceleratedFeedbackFrame
DeviceSpecific/OpenCL/NuiKinfuOpenCLCameraState
DeviceSpecific/OpenCL/NuiKinfuOpenCLColorTracker
DeviceSpecific/OpenCL/NuiKinfuOpenCLDepthTracker.h
DeviceSpecific/OpenCL/NuiKinfuOpenCLFeedbackFrame.h
DeviceSpecific/OpenCL/NuiKinfuOpenCLFrame.h
DeviceSpecific/OpenCL/NuiKinfuOpenCLHashGlobalCache.h
DeviceSpecific/OpenCL/NuiKinfuOpenCLHashScene.h
DeviceSpecific/OpenCL/NuiKinfuOpenCLScene.h
DeviceSpecific/OpenCL/NuiKinfuVoxelBlockHash.h
DeviceSpecific/OpenCL/NuiMarchingCubeTable.h
DeviceSpecific/OpenCL/NuiOpenCLPrefixSum.h
)

set(VO_DEVICE_OPENCL_SOURCES
DeviceSpecific/OpenCL/NuiHashingOpenCLChunkGrid
DeviceSpecific/OpenCL/NuiHashingOpenCLScene
DeviceSpecific/OpenCL/NuiHashingOpenCLSDF
DeviceSpecific/OpenCL/NuiKinfuOpenCLAcceleratedFeedbackFrame.cpp
DeviceSpecific/OpenCL/NuiKinfuOpenCLCameraState.cpp
DeviceSpecific/OpenCL/NuiKinfuOpenCLColorTracker.cpp
DeviceSpecific/OpenCL/NuiKinfuOpenCLDepthTracker.cpp
DeviceSpecific/OpenCL/NuiKinfuOpenCLFeedbackFrame.cpp
DeviceSpecific/OpenCL/NuiKinfuOpenCLFrame.cpp
DeviceSpecific/OpenCL/NuiKinfuOpenCLHashGlobalCache.cpp
DeviceSpecific/OpenCL/NuiKinfuOpenCLHashScene.cpp
DeviceSpecific/OpenCL/NuiKinfuOpenCLScene.cpp
DeviceSpecific/OpenCL/NuiKinfuVoxelBlockHash.cpp
DeviceSpecific/OpenCL/NuiOpenCLPrefixSum.cpp
)

SOURCE_GROUP("" FILES ${VO_HEADERS} ${VO_SOURCES})
SOURCE_GROUP(DeviceSpecific FILES ${VO_DEVICE_HEADERS})
SOURCE_GROUP(DeviceSpecific\\CPU FILES ${VO_DEVICE_CPU_HEADERS} ${VO_DEVICE_CPU_SOURCES})
SOURCE_GROUP(DeviceSpecific\\OPENCL FILES ${VO_DEVICE_OPENCL_HEADERS} ${VO_DEVICE_OPENCL_SOURCES})

add_definitions(-DNOMINMAX)

include_directories($ENV{NUI_LOCATION})
include_directories(${Boost_INCLUDE_DIR})
include_directories($ENV{EIGEN_ROOT})
include_directories($ENV{MAYA_RELEASE_BUILD}/include)

add_library(libVisualOdometry
			${VO_HEADERS}
			${VO_SOURCES}
			${VO_DEVICE_HEADERS}
			${VO_DEVICE_CPU_HEADERS}
			${VO_DEVICE_CPU_SOURCES}
			${VO_DEVICE_OPENCL_HEADERS}
			${VO_DEVICE_OPENCL_SOURCES}
)
target_link_libraries(libVisualOdometry
			${Boost_LIBRARIES}
)

target_link_libraries(libVisualOdometry
					libFoundation
					libShape
					libOpenCLUtilities
					libFrame
)