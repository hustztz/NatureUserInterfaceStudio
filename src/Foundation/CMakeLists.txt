cmake_minimum_required(VERSION 2.6.0)
project(libFoundation)

set(FOUNDATION_HEADERS
NuiBitArray.h
NuiCholesky.h
NuiDebugMacro.h
NuiEventProfiler.h
NuiFileIOUtilities.h
NuiMatrixUtilities.h
NuiProfiler.h
NuiProfilingScope.h
NuiThreadManager.h
NuiThreadObject.h
NuiTimeLog.h
NuiLogger.h
NuiTimeStamp.h
SgVec2T.h
SgVec3T.h
SgVec4T.h
)

set(FOUNDATION_SOURCES
NuiDebugMacro.cpp
NuiEventProfiler.cpp
NuiFileIOUtilities.cpp
NuiMatrixUtilities.cpp
NuiProfiler.cpp
NuiProfilingScope.cpp
NuiThreadObject.cpp
NuiTimeLog.cpp
NuiLogger.cpp
)

include_directories($ENV{NUI_LOCATION})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${log4cplus_DIR}/../../../include)
include_directories(${EIGEN3_ROOT_DIR})

add_library(libFoundation
			${FOUNDATION_HEADERS}
			${FOUNDATION_SOURCES}
)

set(D_SUFF	D)

target_link_libraries(libFoundation
			${Boost_LIBRARIES}
			${log4cplus_DIR}/../../log4cplusU${D_SUFF}.lib
)