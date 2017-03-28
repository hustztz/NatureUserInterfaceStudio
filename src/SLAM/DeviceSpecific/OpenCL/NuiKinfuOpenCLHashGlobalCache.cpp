#include "NuiKinfuOpenCLHashGlobalCache.h"

#include "NuiKinfuVoxelBlockHash.h"
#include "NuiOpenCLPrefixSum.h"

#include "Foundation/NuiDebugMacro.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"

#include <assert.h>

NuiKinfuOpenCLHashGlobalCache::NuiKinfuOpenCLHashGlobalCache()
	: m_swapStates_deviceCL(NULL)
	, m_syncedVoxelBlocks_deviceCL(NULL)
	, m_hasSyncedData_deviceCL(NULL)
	, m_neededEntryIDs_deviceCL(NULL)
{
	AcquireBuffers();
	reset();
}

NuiKinfuOpenCLHashGlobalCache::~NuiKinfuOpenCLHashGlobalCache()
{
	ReleaseBuffers();
}

void	NuiKinfuOpenCLHashGlobalCache::AcquireBuffers()
{
	const UINT nTotalEntries = SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE;
	m_hasStoredData.resize(nTotalEntries);
	m_storedVoxelBlocks.reserve(nTotalEntries * SDF_BLOCK_SIZE3);
	memset(m_storedVoxelBlocks.data(), 0, nTotalEntries * SDF_BLOCK_SIZE3 * sizeof(NuiKinfuVoxel));
	m_swapStates_host.resize(nTotalEntries);

	m_syncedVoxelBlocks_host.reserve(SDF_TRANSFER_BLOCK_NUM * SDF_BLOCK_SIZE3);
	memset(m_syncedVoxelBlocks_host.data(), 0, SDF_TRANSFER_BLOCK_NUM * SDF_BLOCK_SIZE3 * sizeof(NuiKinfuVoxel));
	m_hasSyncedData_host.resize(SDF_TRANSFER_BLOCK_NUM);
	m_neededEntryIDs_host.resize(SDF_TRANSFER_BLOCK_NUM);

	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();

	m_swapStates_deviceCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, nTotalEntries * sizeof(NuiKinfuHashSwapState), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_neededEntryIDs_deviceCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, nTotalEntries * sizeof(cl_int), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_syncedVoxelBlocks_deviceCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, SDF_TRANSFER_BLOCK_NUM * SDF_BLOCK_SIZE3 * sizeof(NuiKinfuVoxel), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_hasSyncedData_deviceCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, SDF_TRANSFER_BLOCK_NUM * sizeof(cl_bool), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_neededEntryFlagsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, nTotalEntries * sizeof(cl_uchar), NULL, &err);
	NUI_CHECK_CL_ERR(err);

	m_pScan = new NuiOpenCLPrefixSum(nTotalEntries);
}

void	NuiKinfuOpenCLHashGlobalCache::ReleaseBuffers()
{
	if (m_swapStates_deviceCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_swapStates_deviceCL);
		NUI_CHECK_CL_ERR(err);
		m_swapStates_deviceCL = NULL;
	}
	if (m_syncedVoxelBlocks_deviceCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_syncedVoxelBlocks_deviceCL);
		NUI_CHECK_CL_ERR(err);
		m_syncedVoxelBlocks_deviceCL = NULL;
	}
	if (m_hasSyncedData_deviceCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_hasSyncedData_deviceCL);
		NUI_CHECK_CL_ERR(err);
		m_hasSyncedData_deviceCL = NULL;
	}
	if (m_neededEntryIDs_deviceCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_neededEntryIDs_deviceCL);
		NUI_CHECK_CL_ERR(err);
		m_neededEntryIDs_deviceCL = NULL;
	}
	if (m_neededEntryFlagsCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_neededEntryFlagsCL);
		NUI_CHECK_CL_ERR(err);
		m_neededEntryFlagsCL = NULL;
	}

	SafeDelete(m_pScan);
}

void	NuiKinfuOpenCLHashGlobalCache::reset()
{
	for (auto i : m_hasStoredData)
		i = 0;

	for (auto i : m_swapStates_host)
		i = 0;

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	const UINT nTotalEntries = SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE;
	NuiKinfuHashSwapState* swapStates = new NuiKinfuHashSwapState[nTotalEntries];
	memset(swapStates, 0, nTotalEntries * sizeof(NuiKinfuHashSwapState));
	err = clEnqueueWriteBuffer(
		queue,
		m_swapStates_deviceCL,
		CL_FALSE,//blocking
		0,
		nTotalEntries * sizeof(NuiKinfuHashSwapState),
		swapStates,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);
	delete[] swapStates;
}


void	NuiKinfuOpenCLHashGlobalCache::reAllocateSwappedOutVoxelBlocks(NuiKinfuVoxelBlockHash* pHashData, cl_mem entriesVisibleTypeCL)
{
	if(!pHashData || !entriesVisibleTypeCL)
		return;

	cl_mem hashEntriesCL = pHashData->getHashEntriesCL();
	cl_mem voxelAllocationListCL = pHashData->getVoxelAllocationListCL();
	cl_mem lastFreeBlockIdCL = pHashData->getLastFreeBlockIdCL();

	if(!hashEntriesCL || !voxelAllocationListCL || !lastFreeBlockIdCL)
		return;

	// Get the kernel
	cl_kernel reallocKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_REALLOC_SWAPPEDOUT_VOXEL);
	assert(reallocKernel);
	if (!reallocKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_BUILD_HASH_ALLOC' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(reallocKernel, idx++, sizeof(cl_mem), &entriesVisibleTypeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(reallocKernel, idx++, sizeof(cl_mem), &hashEntriesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(reallocKernel, idx++, sizeof(cl_mem), &lastFreeBlockIdCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(reallocKernel, idx++, sizeof(cl_mem), &voxelAllocationListCL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	const UINT nTotalEntries = SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE;
	size_t kernelGlobalSize[1] = { nTotalEntries };
	err = clEnqueueNDRangeKernel(
		queue,
		reallocKernel,
		1,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);
}


UINT	NuiKinfuOpenCLHashGlobalCache::BuildSwapInList()
{
	cl_kernel prefixScan1Kernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_PREFIX_SWAPIN_SCAN1);
	assert(prefixScan1Kernel);
	if (!prefixScan1Kernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_PREFIX_SWAPIN_SCAN1' failed!\n");
		return 0;
	}
	cl_kernel prefixScan2Kernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_PREFIX_SWAPIN_SCAN2);
	assert(prefixScan2Kernel);
	if (!prefixScan2Kernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_PREFIX_SWAPIN_SCAN2' failed!\n");
		return 0;
	}
	return m_pScan->prefixSum(prefixScan1Kernel, prefixScan2Kernel, m_swapStates_deviceCL, m_neededEntryIDs_deviceCL);
}

UINT	NuiKinfuOpenCLHashGlobalCache::LoadFromGlobalMemory()
{
	UINT numSwapinList = BuildSwapInList();

	if(numSwapinList > 0)
	{
		numSwapinList = std::min((int)numSwapinList, SDF_TRANSFER_BLOCK_NUM);

		// OpenCL command queue and device
		cl_int           err = CL_SUCCESS;
		cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

		err = clEnqueueReadBuffer(
			queue,
			m_neededEntryIDs_deviceCL,
			CL_TRUE,//blocking
			0, //offset
			numSwapinList * sizeof(cl_int),
			m_neededEntryIDs_host.data(),
			0,
			NULL,
			NULL
			);
		NUI_CHECK_CL_ERR(err);

		memset(m_syncedVoxelBlocks_host.data(), 0, numSwapinList * SDF_BLOCK_SIZE3 * sizeof(NuiKinfuVoxel));
		memset(m_hasSyncedData_host.data(), 0, numSwapinList * sizeof(bool));

		for (UINT i = 0; i < numSwapinList; i++)
		{
			int entryId = m_neededEntryIDs_host.at(i);

			if (m_hasStoredData.at(entryId))
			{
				m_hasSyncedData_host.at(i) = true;
				memcpy((NuiKinfuVoxel*)(m_syncedVoxelBlocks_host.data()) + i * SDF_BLOCK_SIZE3,
					(NuiKinfuVoxel*)(m_storedVoxelBlocks.data()) + entryId * SDF_BLOCK_SIZE3,
					SDF_BLOCK_SIZE3 * sizeof(NuiKinfuVoxel));
			}
		}

		err = clEnqueueWriteBuffer(
			queue,
			m_hasSyncedData_deviceCL,
			CL_FALSE,//blocking
			0, //offset
			numSwapinList * sizeof(cl_bool),
			m_hasSyncedData_host.data(),
			0,
			NULL,
			NULL
			);
		NUI_CHECK_CL_ERR(err);

		err = clEnqueueWriteBuffer(
			queue,
			m_syncedVoxelBlocks_deviceCL,
			CL_FALSE,//blocking
			0, //offset
			numSwapinList * SDF_BLOCK_SIZE3 * sizeof(NuiKinfuVoxel),
			m_syncedVoxelBlocks_host.data(),
			0,
			NULL,
			NULL
			);
		NUI_CHECK_CL_ERR(err);
	}

	return numSwapinList;
}

void	NuiKinfuOpenCLHashGlobalCache::integrateGlobalIntoLocal(
	NuiKinfuVoxelBlockHash* pHashData,
	unsigned char integrationWeightMax)
{
	if(!pHashData)
		return;

	cl_mem hashEntriesCL = pHashData->getHashEntriesCL();
	cl_mem localVoxelBlocksCL = pHashData->getVoxelBlocksCL();
	if(!hashEntriesCL || !localVoxelBlocksCL)
		return;

	UINT nNeededEntries = LoadFromGlobalMemory();

	// Get the kernel
	cl_kernel integrateKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_INTEGRATE_OLD_ACTIVE);
	assert(integrateKernel);
	if (!integrateKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_INTEGRATE_OLD_ACTIVE' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &hashEntriesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &localVoxelBlocksCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &m_neededEntryIDs_deviceCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &m_syncedVoxelBlocks_deviceCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &m_swapStates_deviceCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_uchar), &integrationWeightMax);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	size_t local_ws[1] = {SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE};
	size_t kernelGlobalSize[1] = { nNeededEntries * local_ws[0] };
	err = clEnqueueNDRangeKernel(
		queue,
		integrateKernel,
		1,
		nullptr,
		kernelGlobalSize,
		local_ws,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);
}

UINT	NuiKinfuOpenCLHashGlobalCache::BuildSwapOutList(cl_mem hashEntriesCL, cl_mem entriesVisibleTypeCL)
{
	// Get the kernel
	cl_kernel buildSwapOutKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_BUILD_SWAPOUT_LIST);
	assert(buildSwapOutKernel);
	if (!buildSwapOutKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_BUILD_SWAPOUT_LIST' failed!\n");
		return 0;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(buildSwapOutKernel, idx++, sizeof(cl_mem), &m_neededEntryFlagsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildSwapOutKernel, idx++, sizeof(cl_mem), &m_swapStates_deviceCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildSwapOutKernel, idx++, sizeof(cl_mem), &hashEntriesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildSwapOutKernel, idx++, sizeof(cl_mem), &entriesVisibleTypeCL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	const UINT nTotalEntries = SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE;
	size_t kernelGlobalSize[1] = { nTotalEntries };
	err = clEnqueueNDRangeKernel(
		queue,
		buildSwapOutKernel,
		1,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	return m_pScan->prefixSum(m_neededEntryFlagsCL, m_neededEntryIDs_deviceCL);
}

void	NuiKinfuOpenCLHashGlobalCache::MoveActiveDataToTransferBuffer(UINT nNeededEntries,
																	  cl_mem hashEntriesCL,
																	  cl_mem localVoxelBlocksCL)
{
	// Get the kernel
	cl_kernel transferKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_MOVE_INTO_TRANSFER_BUFFER);
	assert(transferKernel);
	if (!transferKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_MOVE_INTO_TRANSFER_BUFFER' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(transferKernel, idx++, sizeof(cl_mem), &m_syncedVoxelBlocks_deviceCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(transferKernel, idx++, sizeof(cl_mem), &localVoxelBlocksCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(transferKernel, idx++, sizeof(cl_mem), &hashEntriesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(transferKernel, idx++, sizeof(cl_mem), &m_neededEntryIDs_deviceCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(transferKernel, idx++, sizeof(cl_mem), &m_hasSyncedData_deviceCL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	size_t local_ws[1] = {SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE};
	size_t kernelGlobalSize[1] = { nNeededEntries * local_ws[0] };
	err = clEnqueueNDRangeKernel(
		queue,
		transferKernel,
		1,
		nullptr,
		kernelGlobalSize,
		local_ws,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);
}

void	NuiKinfuOpenCLHashGlobalCache::CleanVoxelMemory(
	UINT nNeededEntries,
	cl_mem hashEntriesCL,
	cl_mem voxelAllocationListCL,
	cl_mem lastFreeBlockIdCL)
{
	// Get the kernel
	cl_kernel cleanKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_CLEAN_VOXEL_MEMORY);
	assert(cleanKernel);
	if (!cleanKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_CLEAN_VOXEL_MEMORY' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(cleanKernel, idx++, sizeof(cl_mem), &m_neededEntryIDs_deviceCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(cleanKernel, idx++, sizeof(cl_mem), &hashEntriesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(cleanKernel, idx++, sizeof(cl_mem), &voxelAllocationListCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(cleanKernel, idx++, sizeof(cl_mem), &m_swapStates_deviceCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(cleanKernel, idx++, sizeof(cl_mem), &lastFreeBlockIdCL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	size_t kernelGlobalSize[1] = { nNeededEntries };
	err = clEnqueueNDRangeKernel(
		queue,
		cleanKernel,
		1,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);
}

void	NuiKinfuOpenCLHashGlobalCache::saveToGlobalMemory(NuiKinfuVoxelBlockHash* pHashData, cl_mem entriesVisibleTypeCL)
{
	if(!pHashData || !entriesVisibleTypeCL)
		return;

	cl_mem hashEntriesCL = pHashData->getHashEntriesCL();
	cl_mem localVoxelBlocksCL = pHashData->getVoxelBlocksCL();
	cl_mem voxelAllocationListCL = pHashData->getVoxelAllocationListCL();
	cl_mem lastFreeBlockIdCL = pHashData->getLastFreeBlockIdCL();
	if(!hashEntriesCL || !localVoxelBlocksCL || !voxelAllocationListCL || !lastFreeBlockIdCL)
		return;
	
	UINT numSwapoutList = BuildSwapOutList(hashEntriesCL, entriesVisibleTypeCL);
	if(numSwapoutList > 0)
	{
		numSwapoutList = std::min((int)numSwapoutList, SDF_TRANSFER_BLOCK_NUM);
		MoveActiveDataToTransferBuffer(numSwapoutList, hashEntriesCL, localVoxelBlocksCL);
		CleanVoxelMemory(numSwapoutList, hashEntriesCL, voxelAllocationListCL, lastFreeBlockIdCL);

		// OpenCL command queue and device
		cl_int           err = CL_SUCCESS;
		cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

		err = clEnqueueReadBuffer(
			queue,
			m_neededEntryIDs_deviceCL,
			CL_FALSE,//blocking
			0, //offset
			numSwapoutList * sizeof(cl_int),
			m_neededEntryIDs_host.data(),
			0,
			NULL,
			NULL
			);
		NUI_CHECK_CL_ERR(err);

		err = clEnqueueReadBuffer(
			queue,
			m_hasSyncedData_deviceCL,
			CL_FALSE,//blocking
			0, //offset
			numSwapoutList * sizeof(cl_bool),
			m_hasSyncedData_host.data(),
			0,
			NULL,
			NULL
			);
		NUI_CHECK_CL_ERR(err);

		err = clEnqueueReadBuffer(
			queue,
			m_syncedVoxelBlocks_deviceCL,
			CL_TRUE,//blocking
			0, //offset
			numSwapoutList * SDF_BLOCK_SIZE3 * sizeof(NuiKinfuVoxel),
			m_syncedVoxelBlocks_host.data(),
			0,
			NULL,
			NULL
			);
		NUI_CHECK_CL_ERR(err);

		for (UINT entryId = 0; entryId < numSwapoutList; entryId++)
		{
			if (m_hasSyncedData_host.at(entryId))
			{
				int address = m_neededEntryIDs_host.at(entryId);
				m_hasStoredData.at(address) = true;
				memcpy((NuiKinfuVoxel*)m_storedVoxelBlocks.data() + address * SDF_BLOCK_SIZE3,
					(NuiKinfuVoxel*)m_syncedVoxelBlocks_host.data() + entryId * SDF_BLOCK_SIZE3,
					sizeof(NuiKinfuVoxel) * SDF_BLOCK_SIZE3);
			}
		}
	}
}
