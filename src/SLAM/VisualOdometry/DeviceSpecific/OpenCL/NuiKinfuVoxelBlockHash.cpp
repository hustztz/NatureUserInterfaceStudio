#include "NuiKinfuVoxelBlockHash.h"

#include "Foundation/NuiDebugMacro.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"

#include "Kernels/hashing_gpu_def.h"

#include <assert.h>

NuiKinfuVoxelBlockHash::NuiKinfuVoxelBlockHash()
{
	AcquireBuffers();
	reset();
}

NuiKinfuVoxelBlockHash::~NuiKinfuVoxelBlockHash()
{
	ReleaseBuffers();
}


void NuiKinfuVoxelBlockHash::AcquireBuffers()
{
	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();

	const UINT nTotalEntries = SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE;
	m_hashEntriesCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, nTotalEntries * sizeof(NuiKinfuHashEntry), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_excessAllocationListCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, SDF_EXCESS_LIST_SIZE * sizeof(cl_int), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_lastFreeExcessListIdCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, sizeof(cl_int), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_voxelBlocksCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, SDF_LOCAL_BLOCK_NUM * SDF_BLOCK_SIZE3 * sizeof(NuiKinfuVoxel), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_allocationListCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, SDF_LOCAL_BLOCK_NUM * sizeof(cl_int), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_lastFreeBlockIdCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, sizeof(cl_int), NULL, &err);
	NUI_CHECK_CL_ERR(err);
}

void NuiKinfuVoxelBlockHash::ReleaseBuffers()
{
	if (m_hashEntriesCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_hashEntriesCL);
		NUI_CHECK_CL_ERR(err);
		m_hashEntriesCL = NULL;
	}
	if (m_excessAllocationListCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_excessAllocationListCL);
		NUI_CHECK_CL_ERR(err);
		m_excessAllocationListCL = NULL;
	}
	if (m_lastFreeExcessListIdCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_lastFreeExcessListIdCL);
		NUI_CHECK_CL_ERR(err);
		m_lastFreeExcessListIdCL = NULL;
	}
	if (m_voxelBlocksCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_voxelBlocksCL);
		NUI_CHECK_CL_ERR(err);
		m_voxelBlocksCL = NULL;
	}
	if (m_allocationListCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_allocationListCL);
		NUI_CHECK_CL_ERR(err);
		m_allocationListCL = NULL;
	}
	if (m_lastFreeBlockIdCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_lastFreeBlockIdCL);
		NUI_CHECK_CL_ERR(err);
		m_lastFreeBlockIdCL = NULL;
	}
}

void NuiKinfuVoxelBlockHash::reset()
{
	ResetVoxelBlockBuffer();
	ResetHashEntryBuffer();
}

void NuiKinfuVoxelBlockHash::ResetVoxelBlockBuffer()
{
	// Get the kernel
	cl_kernel initializeKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_INITIALIZE_VOXEL_BLOCK);
	assert(initializeKernel);
	if (!initializeKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_INITIALIZE_HEAP' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(initializeKernel, idx++, sizeof(cl_mem), &m_voxelBlocksCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(initializeKernel, idx++, sizeof(cl_mem), &m_allocationListCL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	size_t kernelGlobalSize[1] = { SDF_LOCAL_BLOCK_NUM };
	err = clEnqueueNDRangeKernel(
		queue,
		initializeKernel,
		1,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	cl_int lastFreeBlockId = SDF_LOCAL_BLOCK_NUM - 1;
	err = clEnqueueWriteBuffer(
		queue,
		m_lastFreeBlockIdCL,
		CL_FALSE,//blocking
		0,
		sizeof(cl_int),
		&lastFreeBlockId,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);
}

void NuiKinfuVoxelBlockHash::ResetHashEntryBuffer()
{
	// Get the kernel
	cl_kernel initializeKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_INITIALIZE_HASHENTRY);
	assert(initializeKernel);
	if (!initializeKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_INITIALIZE_HASH' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(initializeKernel, idx++, sizeof(cl_mem), &m_hashEntriesCL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	const UINT nTotalEntries = SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE;
	size_t kernelGlobalSize[1] = { nTotalEntries };
	err = clEnqueueNDRangeKernel(
		queue,
		initializeKernel,
		1,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	cl_kernel initializeExcessKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_INITIALIZE_EXCESS_ALLOCATION);
	assert(initializeExcessKernel);
	if (!initializeExcessKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_INITIALIZE_EXCESS_ALLOCATION' failed!\n");
		return;
	}

	// Set kernel arguments
	idx = 0;
	err = clSetKernelArg(initializeExcessKernel, idx++, sizeof(cl_mem), &m_excessAllocationListCL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	kernelGlobalSize[0] = SDF_EXCESS_LIST_SIZE;
	err = clEnqueueNDRangeKernel(
		queue,
		initializeExcessKernel,
		1,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	cl_int lastFreeExcessListId = SDF_EXCESS_LIST_SIZE - 1;
	err = clEnqueueWriteBuffer(
		queue,
		m_lastFreeExcessListIdCL,
		CL_FALSE,//blocking
		0,
		sizeof(cl_int),
		&lastFreeExcessListId,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);
}
