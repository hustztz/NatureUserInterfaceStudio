#pragma once

#include "Kernels/hashing_gpu_def.h"

#include "OpenCLUtilities/NuiOpenCLUtil.h"

class NuiKinfuOpenCLHashGlobalCache
{
public:
	NuiKinfuOpenCLHashGlobalCache();
	~NuiKinfuOpenCLHashGlobalCache();

	void			reset();

	cl_mem			getSwapStatesCL() const { return m_swapStates_deviceCL; }

private:
	void			AcquireBuffers();
	void			ReleaseBuffers();
private:
	std::vector<bool>			m_hasStoredData;
	std::vector<NuiKinfuVoxel>	m_storedVoxelBlocks;
	std::vector<NuiKinfuHashSwapState> m_swapStates_host;

	cl_mem		m_syncedVoxelBlocks_hostCL;
	cl_mem		m_hasSyncedData_hostCL;
	cl_mem		m_neededEntryIDs_hostCL;

	cl_mem		m_swapStates_deviceCL;
	cl_mem		m_syncedVoxelBlocks_deviceCL;
	cl_mem		m_hasSyncedData_deviceCL;
	cl_mem		m_neededEntryIDs_deviceCL;
};