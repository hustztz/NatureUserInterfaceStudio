#pragma once

#include "OpenCLUtilities/NuiGPUMemManager.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"

namespace NuiGuiOpenCLUtilities
{
	bool informRenderHoldGPU(unsigned int size, unsigned int* evictedGPUMemSize = nullptr);
	bool informRenderReleaseGPU(unsigned int size);
	void lockResourceHandle(void* ogsBuf, NuiGPUMemSharedType type);
	void unlockResourceHandle(void* ogsBuf, NuiGPUMemSharedType type);
}