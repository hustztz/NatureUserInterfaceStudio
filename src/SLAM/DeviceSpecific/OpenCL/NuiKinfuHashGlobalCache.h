#pragma once

#include "OpenCLUtilities/NuiOpenCLUtil.h"

class NuiKinfuHashGlobalCache
{
public:
	NuiKinfuHashGlobalCache();
	~NuiKinfuHashGlobalCache();

private:
	cl_mem		m_voxelBlocksCL;
	cl_mem		m_allocationListCL;

	UINT		m_lastFreeBlockId;
	UINT		m_allocatedSize;
};