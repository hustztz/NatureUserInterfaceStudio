#include "NuiKinfuOpenCLHashGlobalCache.h"

#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"

NuiKinfuOpenCLHashGlobalCache::NuiKinfuOpenCLHashGlobalCache()
	: m_syncedVoxelBlocks_hostCL(NULL)
	, m_neededEntryIDs_hostCL(NULL)
	, m_swapStates_deviceCL(NULL)
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
	m_storedVoxelBlocks.resize(nTotalEntries * SDF_BLOCK_SIZE3);
	m_swapStates_host.resize(nTotalEntries);

	cl_mem		m_hasSyncedData_hostCL;
	cl_mem		m_neededEntryIDs_hostCL;

	cl_mem		m_swapStates_deviceCL;
	cl_mem		m_syncedVoxelBlocks_deviceCL;
	cl_mem		m_hasSyncedData_deviceCL;
	cl_mem		m_neededEntryIDs_deviceCL;

	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();

	m_syncedVoxelBlocks_hostCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, SDF_TRANSFER_BLOCK_NUM * SDF_BLOCK_SIZE3 * sizeof(NuiKinfuVoxel), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_hasSyncedData_hostCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, SDF_TRANSFER_BLOCK_NUM * sizeof(cl_bool), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_neededEntryIDs_hostCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, SDF_TRANSFER_BLOCK_NUM * sizeof(cl_int), NULL, &err);
	NUI_CHECK_CL_ERR(err);

	m_swapStates_deviceCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, nTotalEntries * sizeof(NuiKinfuHashSwapState), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_syncedVoxelBlocks_deviceCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, SDF_TRANSFER_BLOCK_NUM * SDF_BLOCK_SIZE3 * sizeof(NuiKinfuVoxel), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_hasSyncedData_deviceCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, SDF_TRANSFER_BLOCK_NUM * sizeof(cl_bool), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_neededEntryIDs_deviceCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, SDF_TRANSFER_BLOCK_NUM * sizeof(cl_int), NULL, &err);
	NUI_CHECK_CL_ERR(err);
}

void	NuiKinfuOpenCLHashGlobalCache::ReleaseBuffers()
{
	if (m_syncedVoxelBlocks_hostCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_syncedVoxelBlocks_hostCL);
		NUI_CHECK_CL_ERR(err);
		m_syncedVoxelBlocks_hostCL = NULL;
	}
	if (m_hasSyncedData_hostCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_hasSyncedData_hostCL);
		NUI_CHECK_CL_ERR(err);
		m_hasSyncedData_hostCL = NULL;
	}
	if (m_neededEntryIDs_hostCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_neededEntryIDs_hostCL);
		NUI_CHECK_CL_ERR(err);
		m_neededEntryIDs_hostCL = NULL;
	}
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