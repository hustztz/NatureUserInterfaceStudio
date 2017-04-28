set(DIRS
    DeviceManager
    Foundation
    Frame
    OpenCLUtilities
    Shape
    SLAM
	MayaBridge
)

foreach(d ${DIRS})
    add_subdirectory(${d})
endforeach()

set(SRC_HEADERS
NuiTestUtilities.h
stdafx.h
targetver.h
)

set(SRC_SOURCES
NuiTestUtilities.cpp
stdafx.cpp
)