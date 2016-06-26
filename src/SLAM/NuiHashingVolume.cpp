#include "NuiHashingVolume.h"

#include "Foundation/NuiDebugMacro.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"

#include "Kernels/hashing_gpu_def.h"

#define HASH_BUCKET_SIZE 10

NuiHashingVolume::NuiHashingVolume()
	: m_numIntegratedFrames(0)
{
	AcquireBuffers();
	reset();
}

NuiHashingVolume::~NuiHashingVolume()
{
	ReleaseBuffers();
}

void NuiHashingVolume::reset()
{
	m_numIntegratedFrames = 0;

	m_params.m_rigidTransform.setIdentity();
	m_params.m_rigidTransformInverse.setIdentity();
	m_params.m_numOccupiedBlocks = 0;
	ResetBuffers(/*m_hashData, m_params*/);
}

void NuiHashingVolume::AcquireBuffers()
{
	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();

	m_heapCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_params.m_numSDFBlocks*sizeof(cl_uint), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_heapCountCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, sizeof(cl_uint), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_hashCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_params.m_hashNumBuckets * m_params.m_hashBucketSize * sizeof(NuiHashEntry), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_hashDecisionCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_params.m_hashNumBuckets * m_params.m_hashBucketSize * sizeof(cl_int), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_hashDecisionPrefixCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_params.m_hashNumBuckets * m_params.m_hashBucketSize * sizeof(cl_int), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_hashCompactified = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_params.m_hashNumBuckets * m_params.m_hashBucketSize * sizeof(NuiHashEntry), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_SDFBlocksCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_params.m_numSDFBlocks * m_params.m_SDFBlockSize*m_params.m_SDFBlockSize*m_params.m_SDFBlockSize*sizeof(NuiVoxel), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_hashBucketMutexCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_params.m_hashNumBuckets * sizeof(cl_char), NULL, &err);
	NUI_CHECK_CL_ERR(err);
}

void NuiHashingVolume::ReleaseBuffers()
{
	if (m_heapCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_heapCL);
		NUI_CHECK_CL_ERR(err);
		m_heapCL = NULL;
	}
	if (m_heapCountCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_heapCountCL);
		NUI_CHECK_CL_ERR(err);
		m_heapCountCL = NULL;
	}
	if (m_hashCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_hashCL);
		NUI_CHECK_CL_ERR(err);
		m_hashCL = NULL;
	}
	if (m_hashDecisionCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_hashDecisionCL);
		NUI_CHECK_CL_ERR(err);
		m_hashDecisionCL = NULL;
	}
	if (m_hashDecisionPrefixCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_hashDecisionPrefixCL);
		NUI_CHECK_CL_ERR(err);
		m_hashDecisionPrefixCL = NULL;
	}
	if (m_hashCompactified) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_hashCompactified);
		NUI_CHECK_CL_ERR(err);
		m_hashCompactified = NULL;
	}
	if (m_SDFBlocksCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_SDFBlocksCL);
		NUI_CHECK_CL_ERR(err);
		m_SDFBlocksCL = NULL;
	}
	if (m_hashBucketMutexCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_hashBucketMutexCL);
		NUI_CHECK_CL_ERR(err);
		m_hashBucketMutexCL = NULL;
	}
}

void NuiHashingVolume::ResetHeapBuffer()
{
	// Get the kernel
	cl_kernel initializeKernel = nullptr;
		//NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_INITIALIZE_HEAP);
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
	err = clSetKernelArg(initializeKernel, idx++, sizeof(cl_mem), &m_heapCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(initializeKernel, idx++, sizeof(cl_mem), &m_heapCountCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(initializeKernel, idx++, sizeof(cl_mem), &m_SDFBlocksCL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	size_t kernelGlobalSize[1] = { m_params.m_numSDFBlocks };
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
}

void NuiHashingVolume::ResetHashBuffer()
{
	// Get the kernel
	cl_kernel initializeKernel = nullptr;
		//NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_INITIALIZE_HASH);
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
	err = clSetKernelArg(initializeKernel, idx++, sizeof(cl_mem), &m_hashCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(initializeKernel, idx++, sizeof(cl_mem), &m_hashCompactified);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	size_t kernelGlobalSize[1] = { m_params.m_hashNumBuckets * HASH_BUCKET_SIZE };
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
}

void NuiHashingVolume::ResetHashBucketMutexBuffer()
{
	// Get the kernel
	cl_kernel initializeKernel = nullptr;
		//NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_INITIALIZE_HASH_Bucket_Mutex);
	assert(initializeKernel);
	if (!initializeKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_INITIALIZE_HASH_Bucket_Mutex' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(initializeKernel, idx++, sizeof(cl_mem), &m_hashBucketMutexCL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	size_t kernelGlobalSize[1] = { m_params.m_hashNumBuckets };
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
}

void NuiHashingVolume::ResetBuffers()
{
	ResetHeapBuffer();
	ResetHashBuffer();
	ResetHashBucketMutexBuffer();
}

void NuiHashingVolume::AllocSDFs(UINT nWidth, UINT nHeight, cl_mem floatDepthsCL)
{
	ResetHashBucketMutexBuffer();

	// Get the kernel
	cl_kernel allocKernel = nullptr;
		//NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_ALLOC);
	assert(allocKernel);
	if (!allocKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_ALLOC' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_mem), &floatDepthsCL);
	NUI_CHECK_CL_ERR(err);
	/*err = clSetKernelArg(allocKernel, idx++, sizeof(cl_mem), &m_depthsArrCL[0]);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_mem), &m_gaussianCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(UINT), &m_configuration.filter_radius);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(float), &m_configuration.sigma_depth2_inv_half);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(float), &m_configuration.depth_threshold);
	NUI_CHECK_CL_ERR(err);*/

	// Run kernel to calculate 
	size_t kernelGlobalSize[2] = { nWidth, nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		allocKernel,
		2,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);
}