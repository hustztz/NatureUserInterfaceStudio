#pragma once

#include "OpenCLUtilities/NuiGPUMemManager.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"

namespace NuiMayaOpenCLUtilities
{
    // wrapper for MRenderer::holdGPUMemory( MInt64 sizeInBytes )
    bool informMayaHoldGPU(unsigned int size, unsigned int* evictedGPUMemSize = nullptr);

    // wrapper for MRenderer::releaseGPUMemory( MInt64 sizeInBytes )
    bool informMayaReleaseGPU(unsigned int size);

    // wrapper for 'lockResourceHandle'
    void lockResourceHandle(void* ogsBuf, NuiGPUMemSharedType type);

    // wrapper for 'unlockResourceHandle'
    void unlockResourceHandle(void* ogsBuf, NuiGPUMemSharedType type);
}

