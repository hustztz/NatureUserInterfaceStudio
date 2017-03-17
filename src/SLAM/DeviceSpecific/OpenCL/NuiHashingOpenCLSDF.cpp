#include "NuiHashingOpenCLSDF.h"

#include "NuiOpenCLPrefixSum.h"

#include "Foundation/NuiDebugMacro.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"

#include "Kernels/hashing_gpu_def.h"

#include <assert.h>

NuiHashingOpenCLSDF::NuiHashingOpenCLSDF(NuiHashingSDFConfig config)
	: m_config(config)
	, m_numIntegratedFrames(0)
	, m_bGarbageCollectionEnabled(true)
	, m_garbageCollectionStarve(10)
{
	AcquireBuffers();
	reset();
}

NuiHashingOpenCLSDF::~NuiHashingOpenCLSDF()
{
	ReleaseBuffers();
}

void NuiHashingOpenCLSDF::reset()
{
	m_numIntegratedFrames = 0;

	/*m_params.m_rigidTransform.setIdentity();
	m_params.m_rigidTransformInverse.setIdentity();*/

	ResetHeapBuffer();
	ResetHashBuffer();
	resetHashBucketMutexBuffer();
}

UINT NuiHashingOpenCLSDF::integrate(
	UINT nWidth, UINT nHeight,
	cl_mem floatDepthsCL,
	cl_mem colorsCL,
	cl_mem cameraParamsCL,
	cl_mem transformCL,
	cl_mem bitMaskCL,
	const SgVec3f&	voxelExtends,
	const SgVec3i&	gridDimensions,
	const SgVec3i&	minGridPos
	)
{
	//allocate all hash blocks which are corresponding to depth map entries
	alloc(nWidth, nHeight, floatDepthsCL, cameraParamsCL, transformCL, bitMaskCL, voxelExtends, gridDimensions, minGridPos);

	//generate a linear hash array with only occupied entries
	UINT numOccupiedBlocks = compactifyHashEntries(cameraParamsCL, transformCL);
	if(0 != numOccupiedBlocks)
	{
		//volumetrically integrate the depth data into the depth SDFBlocks
		integrateDepthMap(numOccupiedBlocks, floatDepthsCL, colorsCL, cameraParamsCL, transformCL);

		if(m_bGarbageCollectionEnabled)
		{
			bool bGarbageCollectionStarve = (m_numIntegratedFrames > 0) && (m_numIntegratedFrames % m_garbageCollectionStarve == 0);
			garbageCollect(bGarbageCollectionStarve, numOccupiedBlocks, cameraParamsCL);
		}

		m_numIntegratedFrames++;
	}

	return numOccupiedBlocks;
}

void NuiHashingOpenCLSDF::AcquireBuffers()
{
	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();

	m_heapCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_config.m_numSDFBlocks*sizeof(cl_uint), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_heapCountCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, sizeof(cl_uint), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_hashCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_config.m_hashNumBuckets * HASH_BUCKET_SIZE * sizeof(NuiCLHashEntry), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_hashDecisionCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_config.m_hashNumBuckets * HASH_BUCKET_SIZE * sizeof(cl_uint), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_hashDecisionPrefixCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_config.m_hashNumBuckets * HASH_BUCKET_SIZE * sizeof(cl_uint), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_hashCompactified = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_config.m_hashNumBuckets * HASH_BUCKET_SIZE * sizeof(NuiCLHashEntry), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_SDFBlocksCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_config.m_numSDFBlocks * SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*sizeof(NuiCLVoxel), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_hashBucketMutexCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_config.m_hashNumBuckets * sizeof(cl_uchar), NULL, &err);
	NUI_CHECK_CL_ERR(err);

	m_pScan = new NuiOpenCLPrefixSum(HASH_BUCKET_SIZE * m_config.m_hashNumBuckets);
}

void NuiHashingOpenCLSDF::ReleaseBuffers()
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

	SafeDelete(m_pScan);
}

void NuiHashingOpenCLSDF::ResetHeapBuffer()
{
	// Get the kernel
	cl_kernel initializeKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_INITIALIZE_HEAP);
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
	size_t kernelGlobalSize[1] = { m_config.m_numSDFBlocks };
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

void NuiHashingOpenCLSDF::ResetHashBuffer()
{
	// Get the kernel
	cl_kernel initializeKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_INITIALIZE_HASH);
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
	size_t kernelGlobalSize[1] = { m_config.m_hashNumBuckets * HASH_BUCKET_SIZE };
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

void NuiHashingOpenCLSDF::resetHashBucketMutexBuffer()
{
	// Get the kernel
	cl_kernel initializeKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_INITIALIZE_HASH_Bucket_Mutex);
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
	size_t kernelGlobalSize[1] = { m_config.m_hashNumBuckets };
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

void NuiHashingOpenCLSDF::alloc(
	UINT nWidth, UINT nHeight,
	cl_mem floatDepthsCL,
	cl_mem cameraParamsCL,
	cl_mem transformCL,
	cl_mem bitMaskCL,
	const SgVec3f&	voxelExtends,
	const SgVec3i&	gridDimensions,
	const SgVec3i&	minGridPos
	)
{
	resetHashBucketMutexBuffer();

	// Get the kernel
	cl_kernel allocKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_ALLOC_SDF);
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
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_mem), &transformCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_mem), &m_hashCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_mem), &m_heapCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_mem), &m_heapCountCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_mem), &m_hashBucketMutexCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_mem), &bitMaskCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_float), &m_config.m_truncation);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_float), &m_config.m_truncScale);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_float), &m_config.m_virtualVoxelSize);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_float3), voxelExtends.getValue());
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_int3), minGridPos.getValue());
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_int3), gridDimensions.getValue());
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_uint), &m_config.m_hashNumBuckets);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_uint), &m_config.m_hashMaxCollisionLinkedListSize);
	NUI_CHECK_CL_ERR(err);

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

	// Debug
#ifdef _DEBUG
	//UINT debugHeapCounter = 0;
	//err = clEnqueueReadBuffer(
	//	queue,
	//	m_heapCountCL,
	//	CL_TRUE,//blocking
	//	0, //offset
	//	sizeof(cl_uint),
	//	&debugHeapCounter,
	//	0,
	//	NULL,
	//	NULL
	//	);
	//NUI_CHECK_CL_ERR(err);
	//std::cout << debugHeapCounter  << std::endl;
#endif
}

UINT NuiHashingOpenCLSDF::compactifyHashEntries(cl_mem cameraParamsCL, cl_mem transformCL)
{
	// Get the kernel
	cl_kernel fillDecisionKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_FILL_DECISION_ARRAY);
	assert(fillDecisionKernel);
	if (!fillDecisionKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_FILL_DECISION_ARRAY' failed!\n");
		return 0;
	}

	cl_kernel compacityKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_COMPACTIFY_HASH);
	assert(compacityKernel);
	if (!compacityKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_COMPACTIFY_HASH' failed!\n");
		return 0;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(fillDecisionKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fillDecisionKernel, idx++, sizeof(cl_mem), &transformCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fillDecisionKernel, idx++, sizeof(cl_mem), &m_hashDecisionCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fillDecisionKernel, idx++, sizeof(cl_mem), &m_hashCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fillDecisionKernel, idx++, sizeof(cl_float), &m_config.m_virtualVoxelSize);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	size_t kernelGlobalSize[1] = { HASH_BUCKET_SIZE * m_config.m_hashNumBuckets };
	err = clEnqueueNDRangeKernel(
		queue,
		fillDecisionKernel,
		1,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	// Debug
#ifdef _DEBUG
	//UINT numElements = HASH_BUCKET_SIZE * m_config.m_hashNumBuckets;
	//unsigned int* pBuffer = new unsigned int[numElements];
	//err = clEnqueueReadBuffer(
	//	queue,
	//	m_hashDecisionCL,
	//	CL_TRUE,//blocking
	//	0, //offset
	//	numElements * sizeof(cl_uint),
	//	pBuffer,
	//	0,
	//	NULL,
	//	NULL
	//	);
	//NUI_CHECK_CL_ERR(err);
	//unsigned int hustztz = 0;
	//for(unsigned int i = 0; i < numElements; i++)
	//{
	//	hustztz += pBuffer[i];
	//	//std::cout << pBuffer[i]  << std::endl;
	//}
	//delete[] pBuffer;
	//std::cout << hustztz  << std::endl;
#endif

	//make sure numOccupiedBlocks is updated on the GPU
	UINT numOccupiedBlocks = m_pScan->prefixSum(m_hashDecisionCL, m_hashDecisionPrefixCL);
	numOccupiedBlocks --;

	// Set kernel arguments
	idx = 0;
	err = clSetKernelArg(compacityKernel, idx++, sizeof(cl_mem), &m_hashDecisionCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(compacityKernel, idx++, sizeof(cl_mem), &m_hashDecisionPrefixCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(compacityKernel, idx++, sizeof(cl_mem), &m_hashCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(compacityKernel, idx++, sizeof(cl_mem), &m_hashCompactified);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	err = clEnqueueNDRangeKernel(
		queue,
		compacityKernel,
		1,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	return numOccupiedBlocks;
}

void NuiHashingOpenCLSDF::integrateDepthMap(UINT numOccupiedBlocks, cl_mem floatDepthsCL, cl_mem colorsCL, cl_mem cameraParamsCL, cl_mem transformCL)
{
	if(0 == numOccupiedBlocks)
		return;

	// Get the kernel
	cl_kernel integrateKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_INTEGRATE_DEPTH_MAP);
	assert(integrateKernel);
	if (!integrateKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_INTEGRATE_DEPTH_MAP' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &floatDepthsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &colorsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &transformCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &m_SDFBlocksCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &m_hashCompactified);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_float), &m_config.m_virtualVoxelSize);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_float), &m_config.m_truncation);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_float), &m_config.m_truncScale);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_uint), &m_config.m_integrationWeightSample);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_uchar), &m_config.m_integrationWeightMax);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	size_t local_ws[1] = {SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE}; 
	size_t kernelGlobalSize[1] = { numOccupiedBlocks * local_ws[0] };
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

void NuiHashingOpenCLSDF::garbageCollect(bool bGarbageCollectionStarve, UINT numOccupiedBlocks, cl_mem cameraParamsCL)
{
	// Get the kernel
	cl_kernel starveVoxelsKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_STARVE_VOXELS);
	assert(starveVoxelsKernel);
	if (!starveVoxelsKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_STARVE_VOXELS' failed!\n");
		return;
	}

	cl_kernel grabageIndentifyKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_GARBAGE_COLLECT_INDENTIFY);
	assert(grabageIndentifyKernel);
	if (!grabageIndentifyKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_GARBAGE_COLLECT_INDENTIFY' failed!\n");
		return;
	}

	cl_kernel grabageFreeKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_GARBAGE_COLLECT_FREE);
	assert(grabageFreeKernel);
	if (!grabageFreeKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_GARBAGE_COLLECT_FREE' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	
	if(bGarbageCollectionStarve)
	{
		err = clSetKernelArg(starveVoxelsKernel, idx++, sizeof(cl_mem), &m_SDFBlocksCL);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(starveVoxelsKernel, idx++, sizeof(cl_mem), &m_hashCompactified);
		NUI_CHECK_CL_ERR(err);

		// Run kernel to calculate
		size_t local_ws[1] = {SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE};
		size_t kernelGlobalSize[1] = { numOccupiedBlocks * local_ws[0] };
		err = clEnqueueNDRangeKernel(
			queue,
			starveVoxelsKernel,
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
	
	// Set kernel arguments
	idx = 0;
	err = clSetKernelArg(grabageIndentifyKernel, idx++, sizeof(cl_mem), &m_SDFBlocksCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(grabageIndentifyKernel, idx++, sizeof(cl_mem), &m_hashCompactified);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(grabageIndentifyKernel, idx++, sizeof(cl_mem), &m_hashDecisionCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(grabageIndentifyKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(grabageIndentifyKernel, idx++, sizeof(cl_float), &m_config.m_truncation);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(grabageIndentifyKernel, idx++, sizeof(cl_float), &m_config.m_truncScale);
	NUI_CHECK_CL_ERR(err);

	size_t local_ws[1] = {SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE / 2};
	err = clSetKernelArg(grabageIndentifyKernel, idx++, local_ws[0] * sizeof(cl_float), NULL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(grabageIndentifyKernel, idx++, local_ws[0] * sizeof(cl_uchar), NULL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	size_t kernelGlobalSize[1] = { numOccupiedBlocks * local_ws[0] };
	err = clEnqueueNDRangeKernel(
		queue,
		grabageIndentifyKernel,
		1,
		nullptr,
		kernelGlobalSize,
		local_ws,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	// Set kernel arguments
	idx = 0;
	err = clSetKernelArg(grabageFreeKernel, idx++, sizeof(cl_mem), &m_SDFBlocksCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(grabageFreeKernel, idx++, sizeof(cl_mem), &m_hashCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(grabageFreeKernel, idx++, sizeof(cl_mem), &m_hashCompactified);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(grabageFreeKernel, idx++, sizeof(cl_mem), &m_hashDecisionCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(grabageFreeKernel, idx++, sizeof(cl_mem), &m_heapCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(grabageFreeKernel, idx++, sizeof(cl_mem), &m_heapCountCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(grabageFreeKernel, idx++, sizeof(cl_mem), &m_hashBucketMutexCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(grabageFreeKernel, idx++, sizeof(cl_uint), &m_config.m_hashNumBuckets);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(grabageFreeKernel, idx++, sizeof(cl_uint), &m_config.m_hashMaxCollisionLinkedListSize);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	kernelGlobalSize[0] = numOccupiedBlocks;
	err = clEnqueueNDRangeKernel(
		queue,
		grabageFreeKernel,
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