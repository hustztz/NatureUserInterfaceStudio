cmake_minimum_required(VERSION 2.8)
project(NatureUserInterfaceStudio)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

find_package(OPENCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OPENGL REQUIRED)
find_package(log4cplus REQUIRED)
find_package(EIGEN3 REQUIRED)
find_package(Boost COMPONENTS timer thread REQUIRED)

find_path(TBB_INCLUDE_DIR
NAMES tbb/tbb.h
PATHS C:/Program Files (x86)/IntelSWTools/compilers_and_libraries_2016.3.207/windows/tbb/include/)
find_library(TBB_LIBRARY
NAMES tbb.lib
PATHS C:/Program Files (x86)/IntelSWTools/compilers_and_libraries_2016.3.207/windows/tbb/lib/intel64_win/vc14/)

# include_directories(${EIGEN_INCLUDE_DIRS})
# include_directories($ENV{MAYA_RELEASE_BUILD}\include)


add_definitions(-D_WITH_OPENMP)
if(WIN32)
	add_definitions(-D_WIN32)
	add_definitions(-DWINDOWS)
	IF(MSVC_IDE)
	  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
	  # add_definitions(-DUSING_CMAKE=1)
	ELSE(MSVC_IDE)
	  set(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
	  set(CMAKE_CXX_FLAGS "-fPIC -O3 -march=native ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
	  #set(CMAKE_CXX_FLAGS "-fPIC -g ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
	ENDIF(MSVC_IDE)
endif(WIN32)

add_subdirectory("NatureUserInterfaceApp")
add_subdirectory("src")
