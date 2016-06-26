// Copyright 2015 Autodesk, Inc. All rights reserved. 
//
// Use of this software is subject to the terms of the Autodesk 
// license agreement provided at the time of installation or download, 
// or which otherwise accompanies this software in either electronic 
// or hard copy form.

#ifndef XGMSCULPTUTIL_H
#define XGMSCULPTUTIL_H

#include <clew/clew.h>

#include <vector>

#include "OpenCLUtilities/XgGPUMemManager.h"
#include "OpenCLUtilities/XgOpenCLKernelManager.h"

// class forwards
namespace MHWRender
{
    class MUIDrawManager;
    class MFrameContext;
}
class XgmDescription;
class XgmBrushContext;
class XgmSplineBaseNode;
class MDagPath;
class MObject;
class MString;
class MStringArray;
class MPoint;
class MVector;
class MTypeId;

namespace xgmsculptutil
{
    
    // Fetch data from spline data
    bool populateGeomFromPlug(
        XgmBrushContext* brushCtx,
        std::vector<cl_mem>* primitiveInfos,
        std::vector<cl_mem>* positions,
        std::vector<cl_mem>* positions_final,
        std::vector<cl_mem>* texcoords,
        std::vector<cl_mem>* falloffs,
        std::vector<cl_mem>* rttfalloffs,
        std::vector<cl_mem>* cachedBuffers,
        std::vector<cl_mem>* meshN,
        std::vector<cl_mem>* tweaks,
        std::vector<cl_mem>* originalTweaks,
        std::vector<unsigned int>* primitiveCount,
        std::vector<unsigned int>* vertexCount,
        std::vector<cl_mem>* selectionSet = nullptr,
        std::vector<cl_mem>* frozenSet = nullptr,
        std::vector<cl_mem>* originalFrozenSet = nullptr,
        std::vector<cl_mem>* widthPerSpline = nullptr,
        std::vector<cl_mem>* widthPerCV = nullptr
        );

    // wrapper for MRenderer::holdGPUMemory( MInt64 sizeInBytes )
    bool informMayaHoldGPU(unsigned int size, unsigned int* evictedGPUMemSize = nullptr);

    // wrapper for MRenderer::releaseGPUMemory( MInt64 sizeInBytes )
    bool informMayaReleaseGPU(unsigned int size);

    // wrapper for 'lockResourceHandle'
    void lockResourceHandle(void* ogsBuf, XgGPUMemSharedType type);

    // wrapper for 'unlockResourceHandle'
    void unlockResourceHandle(void* ogsBuf, XgGPUMemSharedType type);
}

#endif
