cmake_minimum_required(VERSION 2.6.0)
project(libFrame)

set(FRAME_BUFFER_HEADS
Buffer/NuiDataPtrCache.h
Buffer/NuiDataPtrCircleBuffer.h
Buffer/NuiDataSharedPtrCache.h
Buffer/NuiDataSharedPtrCircleBuffer.h
Buffer/NuiFrameBuffer.h
Buffer/NuiFrameCache.h
Buffer/NuiFrameCacheImpl.h
Buffer/NuiFrameCircleBuffer.h
)

set(FRAME_BUFFER_SOURCES
Buffer/NuiFrameBuffer.cpp
Buffer/NuiFrameCache.cpp
)

set(FRAME_HEADS
NuiCompositeFrame.h
NuiVisualFrame.h
NuiCompoundImageFrame.h
NuiCompressedDepthFrame.h
NuiExtendedDepthFrame.h
NuiFaceTrackingFrame.h
NuiFacialModelFrame.h
NuiFrameSaveManager.h
NuiVisualFrameSaveManager.h
NuiFrameUtilities.h
NuiFusionImageFrame.h
NuiGestureFrame.h
NuiGrabberFrame.h
NuiImageFrame.h
NuiPointCloudFrame.h
NuiSkeletonFrame.h
)

set(FRAME_SOURCES
NuiCompositeFrame.cpp
NuiVisualFrame.cpp
NuiCompoundImageFrame.cpp
NuiCompressedDepthFrame.cpp
NuiExtendedDepthFrame.cpp
NuiFaceTrackingFrame.cpp
NuiFacialModelFrame.cpp
NuiFrameSaveManager.cpp
NuiVisualFrameSaveManager.cpp
NuiFrameUtilities.cpp
NuiFusionImageFrame.cpp
NuiGestureFrame.cpp
NuiPointCloudFrame.cpp
NuiSkeletonFrame.cpp
)

SOURCE_GROUP("" FILES ${FRAME_HEADS} ${FRAME_SOURCES})
SOURCE_GROUP(BUFFER FILES ${FRAME_BUFFER_HEADS} ${FRAME_BUFFER_SOURCES})

include_directories($ENV{NUI_LOCATION})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
# include_directories(${OPENCL_INCLUDE_DIRS})
include_directories($ENV{MAYA_RELEASE_BUILD}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_COMMON_INCLUDE_DIRS})

add_definitions(-D_UNICODE -DUNICODE)

add_library(libFrame
			${FRAME_BUFFER_HEADS}
			${FRAME_BUFFER_SOURCES}
			${FRAME_HEADS}
			${FRAME_SOURCES}
)

target_link_libraries(libFrame
					libShape
					libOpenCLUtilities
					${OpenCV_LIBS}
)