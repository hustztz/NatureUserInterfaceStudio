cmake_minimum_required(VERSION 2.6.0)
project(NatureUserInterfaceDemo)

set(GUI_HEADERS
GUI/NuiCvVis.h
GUI/NuiGuiController.h
GUI/NuiGuiHWMappable.h
GUI/NuiGuiHWTextureMappable.h
GUI/NuiGuiOpenCLUtilities.h
GUI/NuiPangoCloud.h
GUI/NuiPangoCloudShader.h
GUI/NuiPangoMeshShader.h
GUI/NuiPangoPolygonMeshShader.h
GUI/NuiPangoRayIntervalSplattingShader.h
GUI/NuiPangoTexturedCloudShader.h
GUI/NuiPangoTexturedMeshShader.h
GUI/NuiPangoVis.h
)

set(GUI_SOURCES
GUI/NuiCvVis.cpp
GUI/NuiGuiController.cpp
GUI/NuiGuiOpenCLUtilities.cpp
GUI/NuiPangoCloudShader.cpp
GUI/NuiPangoMeshShader.cpp
GUI/NuiPangoPolygonMeshShader.cpp
GUI/NuiPangoRayIntervalSplattingShader.cpp
GUI/NuiPangoTexturedCloudShader.cpp
GUI/NuiPangoTexturedMeshShader.cpp
GUI/NuiPangoVis.cpp
)

SOURCE_GROUP(GUI FILES ${GUI_HEADERS} ${GUI_SOURCES})

find_package(Pangolin 0.1 REQUIRED)

include_directories($ENV{NUI_LOCATION})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_COMMON_INCLUDE_DIRS})
# include_directories(${OPENCL_INCLUDE_DIRS})
include_directories($ENV{MAYA_RELEASE_BUILD}/include)

add_definitions(${PCL_DEFINITIONS})
add_definitions(-DNOMINMAX)
add_definitions(-D_UNICODE -DUNICODE)


add_executable(NatureUserInterfaceDemo NatureUserInterfaceDemo.cpp ${GUI_HEADERS} ${GUI_SOURCES})

target_link_libraries(NatureUserInterfaceDemo
                      ${Pangolin_LIBRARIES}
					  libDeviceManager
                      libSLAM
)