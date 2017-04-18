cmake_minimum_required(VERSION 2.6.0)
project(libFrame)

set(FRAME_HEADERS
NuiCompositeFrame.h
NuiCompoundImageFrame.h
NuiCompressedDepthFrame.h
NuiExtendedDepthFrame.h
NuiFaceTrackingFrame.h
NuiFacialModelFrame.h
NuiFrameSaveManager.h
NuiFrameUtilities.h
NuiFusionImageFrame.h
NuiGestureFrame.h
NuiGrabberFrame.h
NuiImageFrame.h
NuiPointCloudFrame.h
NuiSkeletonFrame.h
Buffer/NuiDataPtrCache.h
Buffer/NuiDataPtrCircleBuffer.h
Buffer/NuiDataSharedPtrCache.h
Buffer/NuiDataSharedPtrCircleBuffer.h
Buffer/NuiFrameBuffer.h
Buffer/NuiFrameCache.h
Buffer/NuiFrameCacheImpl.h
Buffer/NuiFrameCircleBuffer.h
)

set(FRAME_SOURCES
NuiCompositeFrame.cpp
NuiCompoundImageFrame.cpp
NuiCompressedDepthFrame.cpp
NuiExtendedDepthFrame.cpp
NuiFaceTrackingFrame.cpp
NuiFacialModelFrame.cpp
NuiFrameSaveManager.cpp
NuiFrameUtilities.cpp
NuiFusionImageFrame.cpp
NuiGestureFrame.cpp
NuiPointCloudFrame.cpp
NuiSkeletonFrame.cpp
Buffer/NuiFrameBuffer.cpp
Buffer/NuiFrameCache.cpp
)

include_directories($ENV{NUI_LOCATION})
include_directories(${Boost_INCLUDE_DIR})
include_directories($ENV{EIGEN_ROOT})
# include_directories(${OPENCL_INCLUDE_DIRS})
include_directories($ENV{MAYA_RELEASE_BUILD}/include)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(libFrame
			${FRAME_HEADERS}
			${FRAME_SOURCES}
)
target_link_libraries(libFrame ${Boost_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(libFrame
					libFoundation
					libShape
					libOpenCLUtilities
)