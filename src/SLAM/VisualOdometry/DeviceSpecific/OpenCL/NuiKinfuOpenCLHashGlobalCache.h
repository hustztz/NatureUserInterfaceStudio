#pragma once

#include "Kernels/hashing_gpu_def.h"

#include "OpenCLUtilities/NuiOpenCLUtil.h"

class NuiOpenCLPrefixSum;
class NuiKinfuVoxelBlockHash;

class NuiKinfuOpenCLHashGlobalCache
{
public:
	NuiKinfuOpenCLHashGlobalCache();
	~NuiKinfuOpenCLHashGlobalCache();

	void			reset();

	void			reAllocateSwappedOutVoxelBlocks(NuiKinfuVoxelBlockHash* pHashData, cl_mem entriesVisibleTypeCL);
	void			integrateGlobalIntoLocal(NuiKinfuVoxelBlockHash* pHashData, unsigned char integrationWeightMax);
	void			saveToGlobalMemory(NuiKinfuVoxelBlockHash* pHashData, cl_mem entriesVisibleTypeCL);

	cl_mem			getSwapStatesCL() const { return m_swapStates_deviceCL; }

protected:
	UINT			BuildSwapInList();
	UINT			LoadFromGlobalMemory();
	UINT			BuildSwapOutList(cl_mem hashEntriesCL, cl_mem entriesVisibleTypeCL);
	void			MoveActiveDataToTransferBuffer(
		UINT nNeededEntries,
		cl_mem hashEntriesCL,
		cl_mem localVoxelBlocksCL);
	void			CleanVoxelMemory(
		UINT nNeededEntries,
		cl_mem hashEntriesCL,
		cl_mem voxelAllocationListCL,
		cl_mem lastFreeBlockIdCL);

private:
	void			AcquireBuffers();
	void			ReleaseBuffers();
private:
	std::vector<bool>			m_hasStoredData;
	std::vector<NuiKinfuVoxel>	m_storedVoxelBlocks;
	std::vector<NuiKinfuHashSwapState> m_swapStates_host;

	NuiOpenCLPrefixSum*			m_pScan;

	std::vector<NuiKinfuVoxel>	m_syncedVoxelBlocks_host;
	std::vector<cl_bool>		m_hasSyncedData_host;
	std::vector<cl_int>			m_neededEntryIDs_host;

	cl_mem		m_swapStates_deviceCL;
	cl_mem		m_syncedVoxelBlocks_deviceCL;
	cl_mem		m_hasSyncedData_deviceCL;
	cl_mem		m_neededEntryIDs_deviceCL;

	// TOBE_DELETED
	cl_mem		m_neededEntryFlagsCL;
};