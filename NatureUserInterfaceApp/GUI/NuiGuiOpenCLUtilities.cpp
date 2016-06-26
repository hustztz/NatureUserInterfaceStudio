#include "NuiGuiOpenCLUtilities.h"

namespace NuiGuiOpenCLUtilities
{
	bool informRenderHoldGPU(unsigned int size, unsigned int* evictedGPUMemSize /*= nullptr*/)
	{
		if (evictedGPUMemSize) {
			*evictedGPUMemSize = 0;
		}
		return true;
	}

	bool informRenderReleaseGPU(unsigned int size)
	{
		return true;
	}

	void lockResourceHandle(void* ogsBuf, NuiGPUMemSharedType type)
	{

	}

	void unlockResourceHandle(void* ogsBuf, NuiGPUMemSharedType type)
	{

	}
}