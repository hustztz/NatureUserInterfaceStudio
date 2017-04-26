cmake_minimum_required(VERSION 2.6.0)
project(libDeviceManager)

set(DEVICE_MANAGER_HEADERS
NuiDevicePerfCount.h
NuiRGBDDeviceBufferImpl.h
NuiRGBDDeviceCircleBuffer.h
NuiRGBDDeviceController.h
NuiRGBDDeviceManagerImpl.h
)

set(DEVICE_MANAGER_SOURCES
NuiRGBDDeviceController.cpp
)

set(KINECTV2_HEADERS
KinectV2/NuiKinectV2FusionGrabber.h
KinectV2/NuiKinectV2Manager.h
)

set(KINECTV2_SOURCES
KinectV2/NuiKinectV2FusionGrabber.cpp
KinectV2/NuiKinectV2Manager.cpp
)
set(FILE_LOADER_HEADERS
FileLoader/NuiFrameLoader.h
FileLoader/NuiFrameLoadManager.h
)

set(FILE_LOADER_SOURCES
FileLoader/NuiFrameLoader.cpp
FileLoader/NuiFrameLoadManager.cpp
)

SOURCE_GROUP("" FILES ${DEVICE_MANAGER_HEADERS} ${DEVICE_MANAGER_SOURCES})
SOURCE_GROUP(KinectV2 FILES ${KINECTV2_HEADERS} ${KINECTV2_SOURCES})
SOURCE_GROUP(FileLoader FILES ${FILE_LOADER_HEADERS} ${FILE_LOADER_SOURCES})

include_directories($ENV{NUI_LOCATION})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories($ENV{KINECT_TOOLKIT_DIR}/inc)
include_directories($ENV{KINECTSDK10_DIR}/inc)
include_directories($ENV{KINECTSDK20_DIR}/inc)

add_library(libDeviceManager
			${DEVICE_MANAGER_HEADERS}
			${DEVICE_MANAGER_SOURCES}
			${KINECTV2_HEADERS}
			${KINECTV2_SOURCES}
			${FILE_LOADER_HEADERS}
			${FILE_LOADER_SOURCES}
)
target_link_libraries(libDeviceManager
			${Boost_LIBRARIES}
			$ENV{KINECTSDK10_DIR}/lib/amd64/Kinect10.lib
			$ENV{KINECT_TOOLKIT_DIR}/lib/amd64/FaceTrackLib.lib
			$ENV{KINECT_TOOLKIT_DIR}/lib/amd64/KinectFusion180_64.lib
			$ENV{KINECTSDK20_DIR}/lib/x64/Kinect20.lib
			$ENV{KINECTSDK20_DIR}/lib/x64/Kinect20.Face.lib
			$ENV{KINECTSDK20_DIR}/lib/x64/Kinect20.Fusion.lib
			$ENV{KINECTSDK20_DIR}/lib/x64/Kinect20.VisualGestureBuilder.lib
)