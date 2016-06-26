#include "NuiMayaOpenCLUtilities.h"

#include "Foundation/NuiDebugMacro.h"

#include <assert.h>

#include <maya/MOpenCLInfo.h>
#include <maya/MViewport2Renderer.h>
#include <maya/MHWGeometry.h>

// wrapper for MRenderer::holdGPUMemory( MInt64 sizeInBytes, MInt64* evictedGPUMemSize )
bool NuiMayaOpenCLUtilities::informMayaHoldGPU(unsigned int size, unsigned int* evictedGPUMemSize/* = nullptr*/)
{
    if (evictedGPUMemSize) {
        *evictedGPUMemSize = 0;
    }

    MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
    assert(renderer);
    if (!renderer) {
        return false;
    }

    MInt64 evicted = 0;
    const MStatus result = renderer->holdGPUMemory((MInt64)size, &evicted);
    if (evictedGPUMemSize) {
        *evictedGPUMemSize = (unsigned int)evicted;
    }

    return result == MS::kSuccess;
}

// wrapper for MRenderer::releaseGPUMemory( MInt64 sizeInBytes )
bool NuiMayaOpenCLUtilities::informMayaReleaseGPU(unsigned int size)
{
    MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
    assert(renderer);
    if (!renderer) {
        return false;
    }

    return MS::kSuccess == renderer->releaseGPUMemory((MInt64)size);
}

// wrapper for 'lockResourceHandle'
void NuiMayaOpenCLUtilities::lockResourceHandle(void* ogsBuf, NuiGPUMemSharedType type)
{
    assert(ogsBuf);
    if (!ogsBuf)
        return;

    if (type == NuiGPUMemSharedType::XG_GPUMEM_SHARED_VB) {
        ((MHWRender::MVertexBuffer*)ogsBuf)->lockResourceHandle();
    }
    else if(type == NuiGPUMemSharedType::XG_GPUMEM_SHARED_IB) {
        ((MHWRender::MIndexBuffer*)ogsBuf)->lockResourceHandle();
    }
    else {
        assert(false);
        NUI_ERROR("Unrecognized gpu sharing type!\n");
    }
}

// wrapper for 'unlockResourceHandle'
void NuiMayaOpenCLUtilities::unlockResourceHandle(void* ogsBuf, NuiGPUMemSharedType type)
{
    assert(ogsBuf);
    if (!ogsBuf)
        return;

    if (type == NuiGPUMemSharedType::XG_GPUMEM_SHARED_VB) {
        ((MHWRender::MVertexBuffer*)ogsBuf)->unlockResourceHandle();
    }
    else if (type == NuiGPUMemSharedType::XG_GPUMEM_SHARED_IB) {
        ((MHWRender::MIndexBuffer*)ogsBuf)->unlockResourceHandle();
    }
    else {
        assert(false);
        NUI_ERROR("Unrecognized gpu sharing type!\n");
    }
}


