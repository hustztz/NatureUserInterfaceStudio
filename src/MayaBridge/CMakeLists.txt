cmake_minimum_required(VERSION 2.6.0)
project(NatureUserInterfaceToMaya)

set(MAYA_BRIDGE_SOURCES
NuiMayaProfiler.h
NuiMayaProfiler.cpp
pluginMain.cpp
)

set(MAYA_DEVICE_GRABBER_SOURCES
# DeviceGrabber/moc_NuiMayaCacheTimer.cpp
# DeviceGrabber/NuiMayaCacheTimer.cpp
# DeviceGrabber/NuiMayaCacheTimer.h
DeviceGrabber/NuiMayaDeviceGrabber.cpp
DeviceGrabber/NuiMayaDeviceGrabber.h
DeviceGrabber/NuiMayaPreviewerRenderOverride.cpp
DeviceGrabber/NuiMayaPreviewerRenderOverride.h
)

set(MAYA_HARDWARE_UTILITIES_SOURCES
HardwareUtilities/NuiMayaHWMappable.h
HardwareUtilities/NuiMayaOpenCLUtilities.h
HardwareUtilities/NuiMayaOpenCLUtilities.cpp
HardwareUtilities/NuiMayaOpenGLThreadProvider.cpp
HardwareUtilities/NuiMayaOpenGLThreadProvider.h
)

set(MAYA_POINTCLOUD_PROCESS_SOURCES
PointCloudProcess/NuiMayaMeshData.h
PointCloudProcess/NuiMayaMeshData.cpp
PointCloudProcess/NuiMayaMeshGenerator.cpp
PointCloudProcess/NuiMayaMeshGenerator.h
)

set(MAYA_POINTCLOUD_SHAPE_SOURCES
PointCloudShape/NuiMayaMappableData.h
PointCloudShape/NuiMayaMappableData.cpp
PointCloudShape/NuiMayaMappableDataIterator.cpp
PointCloudShape/NuiMayaMappableDataIterator.h
PointCloudShape/NuiMayaPointCloudGeometryOverride.cpp
PointCloudShape/NuiMayaPointCloudGeometryOverride.h
PointCloudShape/NuiMayaPointCloudShape.cpp
PointCloudShape/NuiMayaPointCloudShape.h
)

set(MAYA_SKELETON_DRIVER_SOURCES
SkeletonDriver/NuiMayaFacialModelData.h
SkeletonDriver/NuiMayaFacialModelData.cpp
SkeletonDriver/NuiMayaFacialModelDriver.cpp
SkeletonDriver/NuiMayaFacialModelDriver.h
SkeletonDriver/NuiMayaGestureData.cpp
SkeletonDriver/NuiMayaGestureData.h
SkeletonDriver/NuiMayaImageData.cpp
SkeletonDriver/NuiMayaImageData.h
SkeletonDriver/NuiMayaSkeletonData.cpp
SkeletonDriver/NuiMayaSkeletonData.h
SkeletonDriver/NuiMayaSkeletonDriver.cpp
SkeletonDriver/NuiMayaSkeletonDriver.h
)

SOURCE_GROUP(DeviceGrabber FILES ${MAYA_DEVICE_GRABBER_SOURCES})
SOURCE_GROUP(HardwareUtilities FILES ${MAYA_HARDWARE_UTILITIES_SOURCES})
SOURCE_GROUP(PointCloudProcess FILES ${MAYA_POINTCLOUD_PROCESS_SOURCES})
SOURCE_GROUP(PointCloudShape FILES ${MAYA_POINTCLOUD_SHAPE_SOURCES})
SOURCE_GROUP(SkeletonDriver FILES ${MAYA_SKELETON_DRIVER_SOURCES})

add_definitions(-DNOMINMAX)
add_definitions(-D_UNICODE -DUNICODE)
add_definitions(-DOSWin_)

find_package(Maya REQUIRED)

include_directories($ENV{NUI_LOCATION})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
# include_directories(${OPENCL_INCLUDE_DIRS})
include_directories(${PCL_COMMON_INCLUDE_DIRS})
include_directories(${MAYA_INCLUDE_DIRS})
include_directories(${MAYA_INCLUDE_DIR})

add_library(NatureUserInterfaceToMaya SHARED
					  ${MAYA_BRIDGE_SOURCES}
					  ${MAYA_DEVICE_GRABBER_SOURCES}
					  ${MAYA_HARDWARE_UTILITIES_SOURCES}
					  ${MAYA_POINTCLOUD_PROCESS_SOURCES}
					  ${MAYA_POINTCLOUD_SHAPE_SOURCES}
					  ${MAYA_SKELETON_DRIVER_SOURCES}
)

set(D_SUFF	D)

target_link_libraries(NatureUserInterfaceToMaya
					  libDeviceManager
					  libSLAM
					  ${MAYA_Foundation_LIBRARY}
					  ${MAYA_DependEngine_LIBRARY}
					${MAYA_OpenMayaAnim_LIBRARY}
					${MAYA_OpenMayaFX_LIBRARY}
					${MAYA_OpenMayaUI_LIBRARY}
					${MAYA_OpenMaya_LIBRARY}
					${MAYA_OpenMayaRender_LIBRARY}
)