cmake_minimum_required(VERSION 2.6.0)
project(libOpenCLUtilities)

set(OPENCL_UTILITIES_HEADERS
NuiGPUMemManager.h
NuiMappable.h
NuiOfflineRenderFactory.h
NuiOpenCLBufferFactory.h
NuiOpenCLFoundationUtils.h
NuiOpenCLGlobal.h
NuiOpenCLKernelManager.h
NuiOpenCLUtil.h
NuiOpenGLThread.h
NuiTextureMappable.h
)

set(OPENCL_UTILITIES_SOURCES
NuiGPUMemManager.cpp
NuiOfflineRenderFactory.cpp
NuiOpenCLBufferFactory.cpp
NuiOpenCLFoundationUtils.cpp
NuiOpenCLGlobal.cpp
NuiOpenCLKernelManager.cpp
NuiOpenCLUtil.cpp
NuiOpenGLThread.cpp
)

include_directories(../Kernels)

SOURCE_GROUP(Kernels FILES ${OPENCL_KERNELS_HEADERS} ${OPENCL_KERNELS_SOURCES})
SOURCE_GROUP(Kernels/Hash FILES ${OPENCL_KERNELS_HASH_SOURCES})


include_directories($ENV{NUI_LOCATION})
# include_directories(${OPENCL_INCLUDE_DIRS})
include_directories($ENV{MAYA_RELEASE_BUILD}/include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library(libOpenCLUtilities
			${OPENCL_UTILITIES_HEADERS}
			${OPENCL_UTILITIES_SOURCES}
)
target_link_libraries(libOpenCLUtilities
			libFoundation
)

set(MAYA_OPENCL_LIBRARY
	$ENV{MAYA_RELEASE_BUILD}/runTime/lib/tbb_debug.lib
	$ENV{MAYA_RELEASE_BUILD}/runTime/lib/clew.lib
)
target_link_libraries(libOpenCLUtilities
			${MAYA_OPENCL_LIBRARY}
			${OPENGL_LIBRARIES}
)