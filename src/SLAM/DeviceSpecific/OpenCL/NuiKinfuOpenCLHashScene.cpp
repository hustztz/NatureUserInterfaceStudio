#include "NuiKinfuOpenCLHashScene.h"

#include "NuiKinfuOpenCLHashGlobalCache.h"
#include "NuiKinfuOpenCLFrame.h"
#include "NuiKinfuOpenCLAcceleratedFeedbackFrame.h"
#include "NuiKinfuOpenCLCameraState.h"

#include "Kernels/hashing_gpu_def.h"
#include "Kernels/gpu_def.h"
#include "Foundation/NuiDebugMacro.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"
#include "OpenCLUtilities/NuiOpenCLBufferFactory.h"
#include "OpenCLUtilities/NuiOfflineRenderFactory.h"

#include "Shape/NuiCLMappableData.h"

NuiKinfuOpenCLHashScene::NuiKinfuOpenCLHashScene(const NuiHashingSDFConfig& sdfConfig, const NuiHashingRaycastConfig& raycastConfig)
	: m_pGlobalCache(NULL)
	, m_entriesAllocTypeCL(NULL)
	, m_blockCoordsCL(NULL)
	, m_visibleEntryIDsCL(NULL)
	, m_entriesVisibleTypeCL(NULL)
	, m_numVisibleEntries(0)

	, m_raycastConfig(raycastConfig)
{
	reset();
	AcquireBuffers();
}

NuiKinfuOpenCLHashScene::~NuiKinfuOpenCLHashScene()
{
	SafeDelete(m_pGlobalCache);
	ReleaseBuffers();
}

void NuiKinfuOpenCLHashScene::AcquireBuffers()
{
	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();

	const UINT nTotalEntries = SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE;
	m_entriesAllocTypeCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, nTotalEntries * sizeof(cl_uchar), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_blockCoordsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, nTotalEntries * sizeof(cl_short3), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_visibleEntryIDsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, SDF_LOCAL_BLOCK_NUM * sizeof(cl_int), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_entriesVisibleTypeCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, nTotalEntries * sizeof(cl_uchar), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_entriesVisibleTypePrefixCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, nTotalEntries * sizeof(cl_uint), NULL, &err);
	NUI_CHECK_CL_ERR(err);
}


void NuiKinfuOpenCLHashScene::ReleaseBuffers()
{
	if (m_entriesAllocTypeCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_entriesAllocTypeCL);
		NUI_CHECK_CL_ERR(err);
		m_entriesAllocTypeCL = NULL;
	}
	if (m_blockCoordsCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_blockCoordsCL);
		NUI_CHECK_CL_ERR(err);
		m_blockCoordsCL = NULL;
	}
	if (m_visibleEntryIDsCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_visibleEntryIDsCL);
		NUI_CHECK_CL_ERR(err);
		m_visibleEntryIDsCL = NULL;
	}
	if (m_entriesVisibleTypeCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_entriesVisibleTypeCL);
		NUI_CHECK_CL_ERR(err);
		m_entriesVisibleTypeCL = NULL;
	}
	if (m_entriesVisibleTypePrefixCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_entriesVisibleTypePrefixCL);
		NUI_CHECK_CL_ERR(err);
		m_entriesVisibleTypePrefixCL = NULL;
	}
}

bool NuiKinfuOpenCLHashScene::log(const std::string& fileName) const
{
	return true;
}

void	NuiKinfuOpenCLHashScene::reset()
{
	m_hashingVoxelData.reset();
	if(m_pGlobalCache)
		m_pGlobalCache->reset();
	m_numVisibleEntries = 0;

	setDirty();
}

void	NuiKinfuOpenCLHashScene::ResetVisibleEntrys()
{
	if (0 == m_numVisibleEntries)
		return;

	// Get the kernel
	cl_kernel resetKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_RESET_VISIBLE_ENTRYS);
	assert(resetKernel);
	if (!resetKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_RESET_VISIBLE_ENTRYS' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(resetKernel, idx++, sizeof(cl_mem), &m_visibleEntryIDsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(resetKernel, idx++, sizeof(cl_mem), &m_entriesVisibleTypeCL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	size_t kernelGlobalSize[1] = { m_numVisibleEntries };
	err = clEnqueueNDRangeKernel(
		queue,
		resetKernel,
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

void	NuiKinfuOpenCLHashScene::ResetAllocType()
{
	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Run kernel to calculate
	const UINT nTotalEntries = SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE;
	cl_uchar* entriesAllocType = new cl_uchar[nTotalEntries];
	memset(entriesAllocType, 0, nTotalEntries * sizeof(cl_uchar));
	err = clEnqueueWriteBuffer(
		queue,
		m_entriesAllocTypeCL,
		CL_FALSE,//blocking
		0,
		nTotalEntries * sizeof(cl_uchar),
		entriesAllocType,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);
	delete[] entriesAllocType;
}

void	NuiKinfuOpenCLHashScene::BuildHashAllocAndVisibleType(UINT nWidth, UINT nHeight,
															  cl_mem floatDepthsCL,
															  cl_mem cameraParamsCL,
															  cl_mem transformCL)
{
	// Get the kernel
	cl_kernel buildHashKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_BUILD_HASH_ALLOC);
	assert(buildHashKernel);
	if (!buildHashKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_BUILD_HASH_ALLOC' failed!\n");
		return;
	}

	cl_mem hashEntriesCL = m_hashingVoxelData.getHashEntriesCL();
	float oneOverVoxelSize = 1.0f / (m_config.m_virtualVoxelSize * SDF_BLOCK_SIZE);

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(buildHashKernel, idx++, sizeof(cl_mem), &floatDepthsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildHashKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildHashKernel, idx++, sizeof(cl_mem), &transformCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildHashKernel, idx++, sizeof(cl_mem), &hashEntriesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildHashKernel, idx++, sizeof(cl_mem), &m_entriesVisibleTypeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildHashKernel, idx++, sizeof(cl_mem), &m_entriesAllocTypeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildHashKernel, idx++, sizeof(cl_mem), &m_blockCoordsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildHashKernel, idx++, sizeof(cl_float), &m_config.m_truncScale);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildHashKernel, idx++, sizeof(cl_float), &oneOverVoxelSize);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	size_t kernelGlobalSize[2] = { nWidth, nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		buildHashKernel,
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

void	NuiKinfuOpenCLHashScene::AllocateVoxelBlocksList()
{
	// Get the kernel
	cl_kernel allocKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_ALLOC_VOXEL_BLOCKS);
	assert(allocKernel);
	if (!allocKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_ALLOC_VOXEL_BLOCKS' failed!\n");
		return;
	}

	cl_mem hashEntriesCL = m_hashingVoxelData.getHashEntriesCL();
	cl_mem excessAllocationListCL = m_hashingVoxelData.getExcessAllocationListCL();
	cl_mem lastFreeExcessListIdCL = m_hashingVoxelData.getLastFreeExcessListIdCL();
	cl_mem voxelAllocationListCL = m_hashingVoxelData.getVoxelAllocationListCL();
	cl_mem lastFreeBlockIdCL = m_hashingVoxelData.getLastFreeBlockIdCL();

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_mem), &m_entriesAllocTypeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_mem), &hashEntriesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_mem), &excessAllocationListCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_mem), &lastFreeExcessListIdCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_mem), &voxelAllocationListCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_mem), &lastFreeBlockIdCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_mem), &m_blockCoordsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(allocKernel, idx++, sizeof(cl_mem), &m_entriesVisibleTypeCL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	const UINT nTotalEntries = SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE;
	size_t kernelGlobalSize[1] = { nTotalEntries };
	err = clEnqueueNDRangeKernel(
		queue,
		allocKernel,
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

void	NuiKinfuOpenCLHashScene::BuildVisibleList(cl_mem cameraParamsCL, cl_mem transformCL)
{
	// Get the kernel
	cl_kernel buildVisibleKernel = NuiOpenCLKernelManager::instance().acquireKernel(
			m_pGlobalCache ? E_HASHING_BUILD_VISIBLE_ENLARGED_LIST : E_HASHING_BUILD_VISIBLE_LIST);
	assert(buildVisibleKernel);
	if (!buildVisibleKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_BUILD_HASH_ALLOC' failed!\n");
		return;
	}

	cl_mem hashEntriesCL = m_hashingVoxelData.getHashEntriesCL();

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(buildVisibleKernel, idx++, sizeof(cl_mem), &m_entriesVisibleTypeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildVisibleKernel, idx++, sizeof(cl_mem), &hashEntriesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildVisibleKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildVisibleKernel, idx++, sizeof(cl_mem), &transformCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildVisibleKernel, idx++, sizeof(cl_float), &m_config.m_virtualVoxelSize);
	NUI_CHECK_CL_ERR(err);
	if(m_pGlobalCache)
	{
		cl_mem swapStatesCL = m_pGlobalCache->getSwapStatesCL();
		err = clSetKernelArg(buildVisibleKernel, idx++, sizeof(cl_mem), &swapStatesCL);
		NUI_CHECK_CL_ERR(err);
	}

	// Run kernel to calculate
	const UINT nTotalEntries = SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE;
	size_t kernelGlobalSize[1] = { nTotalEntries };
	err = clEnqueueNDRangeKernel(
		queue,
		buildVisibleKernel,
		1,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	m_numVisibleEntries = m_scan.prefixSum(nTotalEntries, m_entriesVisibleTypeCL, m_entriesVisibleTypePrefixCL, true);
	if(0 == m_numVisibleEntries)
		return;
	m_numVisibleEntries --;

	cl_kernel compacityKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_COMPACTIFY_VISIBLE_TYPE);
	assert(compacityKernel);
	if (!compacityKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_COMPACTIFY_HASH' failed!\n");
		return;
	}

	// Set kernel arguments
	idx = 0;
	err = clSetKernelArg(compacityKernel, idx++, sizeof(cl_mem), &m_entriesVisibleTypeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(compacityKernel, idx++, sizeof(cl_mem), &m_entriesVisibleTypePrefixCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(compacityKernel, idx++, sizeof(cl_mem), &m_visibleEntryIDsCL);
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
}

void	NuiKinfuOpenCLHashScene::ReAllocateSwappedOutVoxelBlocks()
{
	// Get the kernel
	cl_kernel reallocKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_REALLOC_SWAPPEDOUT_VOXEL);
	assert(reallocKernel);
	if (!reallocKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_BUILD_HASH_ALLOC' failed!\n");
		return;
	}

	cl_mem hashEntriesCL = m_hashingVoxelData.getHashEntriesCL();
	cl_mem voxelAllocationListCL = m_hashingVoxelData.getVoxelAllocationListCL();
	cl_mem lastFreeBlockIdCL = m_hashingVoxelData.getLastFreeBlockIdCL();

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(reallocKernel, idx++, sizeof(cl_mem), &m_entriesVisibleTypeCL);
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

void	NuiKinfuOpenCLHashScene::AllocateSceneFromDepth(
	UINT nWidth, UINT nHeight,
	cl_mem floatDepthsCL,
	cl_mem cameraParamsCL,
	cl_mem transformCL)
{
	ResetVisibleEntrys();
	ResetAllocType();

	BuildHashAllocAndVisibleType(
		nWidth, nHeight,
		floatDepthsCL,
		cameraParamsCL,
		transformCL);
	AllocateVoxelBlocksList();
	BuildVisibleList(cameraParamsCL, transformCL);
	ReAllocateSwappedOutVoxelBlocks();
}

void	NuiKinfuOpenCLHashScene::IntegrateIntoScene(
	cl_mem floatDepthsCL,
	cl_mem colorsCL,
	cl_mem cameraParamsCL,
	cl_mem transformCL)
{
	// Get the kernel
	cl_kernel integrateKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_INTEGRATE_INTO_SCENE);
	assert(integrateKernel);
	if (!integrateKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_INTEGRATE_INTO_SCENE' failed!\n");
		return;
	}

	cl_mem hashEntriesCL = m_hashingVoxelData.getHashEntriesCL();
	cl_mem voxelBlocksCL = m_hashingVoxelData.getVoxelBlocksCL();

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &m_visibleEntryIDsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &hashEntriesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &voxelBlocksCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &floatDepthsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &colorsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &transformCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_float), &m_config.m_virtualVoxelSize);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_float), &m_config.m_truncation);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_uchar), &m_config.m_integrationWeightMax);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	size_t local_ws[1] = {SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE};
	size_t kernelGlobalSize[1] = { m_numVisibleEntries * local_ws[0] };
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

bool	NuiKinfuOpenCLHashScene::integrateVolume(
	NuiKinfuFrame*			pFrame,
	NuiKinfuCameraState*	pCameraState)
{
	if(!pCameraState)
		return false;
	NuiKinfuOpenCLCameraState* pCLCamera = dynamic_cast<NuiKinfuOpenCLCameraState*>(pCameraState);
	if(!pCLCamera)
		return false;
	cl_mem transformCL = pCLCamera->GetCameraTransformBuffer();
	cl_mem cameraParamsCL = pCLCamera->GetCameraParamsBuffer();
	if(!cameraParamsCL || !transformCL)
		return false;

	if(!pFrame)
		return false;
	NuiKinfuOpenCLFrame* pCLFrame = dynamic_cast<NuiKinfuOpenCLFrame*>(pFrame);
	if(!pCLFrame)
		return false;
	cl_mem floatDepthsCL = pCLFrame->GetDepthBuffer();
	if(!floatDepthsCL)
		return false;

	cl_mem colorsCL = pCLFrame->GetColorBuffer();
	UINT nWidth = pCLFrame->GetWidth();
	UINT nHeight = pCLFrame->GetHeight();

	AllocateSceneFromDepth(
		nWidth, nHeight,
		floatDepthsCL,
		cameraParamsCL,
		transformCL);

	if(m_numVisibleEntries > 0)
	{
		IntegrateIntoScene(
			floatDepthsCL,
			colorsCL,
			cameraParamsCL,
			transformCL);
		setDirty();
	}

	return true;
}

void	NuiKinfuOpenCLHashScene::updateGlobalCacheConfig(const NuiHashingChunkGridConfig& chunkGridConfig)
{
	if(chunkGridConfig.m_enable)
	{
		if(!m_pGlobalCache)
			m_pGlobalCache = new NuiKinfuOpenCLHashGlobalCache();
		//m_pGlobalCache->updateConfig(chunkGridConfig);
	}
	else
	{
		SafeDelete(m_pGlobalCache);
	}
}

void	NuiKinfuOpenCLHashScene::CreateExpectedDepths(
	NuiKinfuFeedbackFrame*	pFeedbackFrame,
	NuiKinfuCameraState*	pCameraState)
{
	NuiKinfuOpenCLAcceleratedFeedbackFrame* pCLFeedbackFrame = dynamic_cast<NuiKinfuOpenCLAcceleratedFeedbackFrame*>(pFeedbackFrame);
	if(!pCLFeedbackFrame)
		return;
	pCLFeedbackFrame->resetExpectedRange();

	if(!pCameraState)
		return;
	NuiKinfuOpenCLCameraState* pCLCamera = dynamic_cast<NuiKinfuOpenCLCameraState*>(pCameraState);
	if(!pCLCamera)
		return;
	cl_mem transformCL = pCLCamera->GetCameraTransformBuffer();
	cl_mem cameraParamsCL = pCLCamera->GetCameraParamsBuffer();
	if(!cameraParamsCL || !transformCL)
		return;

	// Get the kernel
	cl_kernel projectBlocksKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_PROJECT_SPLIT_BLOCKS);
	assert(projectBlocksKernel);
	if (!projectBlocksKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_PROJECT_SPLIT_BLOCKS' failed!\n");
		return;
	}

	cl_mem hashEntriesCL = m_hashingVoxelData.getHashEntriesCL();
	cl_mem voxelBlocksCL = m_hashingVoxelData.getVoxelBlocksCL();

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(projectBlocksKernel, idx++, sizeof(cl_mem), &m_visibleEntryIDsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(projectBlocksKernel, idx++, sizeof(cl_mem), &hashEntriesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(projectBlocksKernel, idx++, sizeof(cl_mem), &voxelBlocksCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(projectBlocksKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(projectBlocksKernel, idx++, sizeof(cl_mem), &transformCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(projectBlocksKernel, idx++, sizeof(cl_float), &m_config.m_virtualVoxelSize);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	size_t kernelGlobalSize[1] = { m_numVisibleEntries };
	err = clEnqueueNDRangeKernel(
		queue,
		projectBlocksKernel,
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


void NuiKinfuOpenCLHashScene::raycastRender(
	NuiKinfuFeedbackFrame*	pFeedbackFrame,
	NuiKinfuCameraState*	pCameraState
	)
{
	if(!pCameraState)
		return;
	NuiKinfuOpenCLCameraState* pCLCamera = dynamic_cast<NuiKinfuOpenCLCameraState*>(pCameraState);
	if(!pCLCamera)
		return ;

	if(!pFeedbackFrame)
		return;
	NuiKinfuOpenCLFeedbackFrame* pCLFeedbackFrame = dynamic_cast<NuiKinfuOpenCLFeedbackFrame*>(pFeedbackFrame);
	if(!pCLFeedbackFrame)
		return;

	m_cameraParamsCL = pCLCamera->GetCameraParamsBuffer();
	m_transformCL = pCLCamera->GetCameraTransformBuffer();
	m_sensorDepthMin = pCameraState->GetCameraPos().getSensorDepthMin();
	m_sensorDepthMax = pCameraState->GetCameraPos().getSensorDepthMax();
	m_renderVerticesCL = pCLFeedbackFrame->GetVertexBuffer();
	m_renderNormalsCL = pCLFeedbackFrame->GetNormalBuffer();
	m_renderIntensitiesCL = NULL;
	m_nWidth = pFeedbackFrame->GetWidth();
	m_nHeight = pFeedbackFrame->GetHeight();


}

bool NuiKinfuOpenCLHashScene::Volume2CLVertices(NuiCLMappableData* pCLData)
{
	assert(pCLData);
	if(!pCLData)
		return false;

	if(!m_dirty)
		return false;
	m_dirty = false;

	const NuiHashingSDFConfig& hashParams = m_pSDFData->getConfig();

	// Set bounding box
	/*const Vector3f& voxelSizeMeters = getVoxelSize();
	pCLData->SetBoundingBox(SgVec3f(
		m_voxel_offset(0)*voxelSizeMeters(0)-m_config.dimensions[0]/2,
		m_voxel_offset(1)*voxelSizeMeters(1)-m_config.dimensions[1]/2,
		m_voxel_offset(2)*voxelSizeMeters(2)-m_config.dimensions[2]/2),
		SgVec3f(
		m_voxel_offset(0)*voxelSizeMeters(0)+m_config.dimensions[0]/2,
		m_voxel_offset(1)*voxelSizeMeters(1)+m_config.dimensions[1]/2,
		m_voxel_offset(2)*voxelSizeMeters(2)+m_config.dimensions[2]/2));*/

	cl_kernel fetchKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_FETCH_VOLUME);
	assert(fetchKernel);
	if (!fetchKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_FETCH_VOLUME' failed!\n");
		return false;
	}

	cl_mem hashCL = m_pSDFData->getHashCL();
	cl_mem SDFBlocksCL = m_pSDFData->getSDFBlocksCL();
	float thresSDF = 0.001f;

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	cl_int vertex_sum = 0;
	err = clEnqueueWriteBuffer(
		queue,
		m_vertexSumCL,
		CL_FALSE,//blocking
		0,
		sizeof(cl_int),
		&vertex_sum,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	// 
	err = clFinish(queue);
	NUI_CHECK_CL_ERR(err);

	if( MAX_OUTPUT_VERTEX_SIZE != pCLData->PositionStream().size() )
	{
		NuiMappableAccessor::asVectorImpl(pCLData->PositionStream())->data().resize(MAX_OUTPUT_VERTEX_SIZE);
	}
	if( MAX_OUTPUT_VERTEX_SIZE != pCLData->ColorStream().size() )
	{
		NuiMappableAccessor::asVectorImpl(pCLData->ColorStream())->data().resize(MAX_OUTPUT_VERTEX_SIZE);
	}

	cl_mem positionsGL = NuiOpenCLBufferFactory::asPosition3fBufferCL(pCLData->PositionStream());
	cl_mem colorsGL = NuiOpenCLBufferFactory::asColor4fBufferCL(pCLData->ColorStream());
	// Acquire OpenGL objects before use
	cl_mem glObjs[] = {
		positionsGL,
		colorsGL
	};

	openclutil::enqueueAcquireHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &hashCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &SDFBlocksCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_float), &hashParams.m_virtualVoxelSize);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_float), &thresSDF);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &positionsGL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &colorsGL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &m_vertexSumCL);
	NUI_CHECK_CL_ERR(err);

#ifdef _GPU_PROFILER
	cl_event timing_event;
	cl_ulong time_start, time_end;
#endif
	// Run kernel to calculate 
	size_t local_ws[1] = {SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE};
	size_t kernelGlobalSize[1] = { hashParams.m_hashNumBuckets * HASH_BUCKET_SIZE * local_ws[0] };
	err = clEnqueueNDRangeKernel(
		queue,
		fetchKernel,
		1,
		nullptr,
		kernelGlobalSize,
		local_ws,
		0,
		NULL,
#ifdef _GPU_PROFILER
		&timing_event
#else
		NULL
#endif
		);
	NUI_CHECK_CL_ERR(err);

	err = clFinish(queue);
	NUI_CHECK_CL_ERR(err);

	// Release OpenGL objects
	openclutil::enqueueReleaseHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);

#ifdef _GPU_PROFILER
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_START, sizeof(time_start), &time_start, NULL);
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_END, sizeof(time_end), &time_end, NULL);
	std::cout << "volume traversal:" << (time_end - time_start) << std::endl;
	clReleaseEvent(timing_event);
#endif

	err = clEnqueueReadBuffer(
		queue,
		m_vertexSumCL,
		CL_TRUE,//blocking
		0,
		sizeof(cl_int),
		&vertex_sum,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	if(vertex_sum <= 0)
		return false;

	NuiMappableAccessor::asVectorImpl(pCLData->TriangleIndices())->data().clear();
	NuiMappableAccessor::asVectorImpl(pCLData->WireframeIndices())->data().clear();

	std::vector<unsigned int>& clPointIndices =
		NuiMappableAccessor::asVectorImpl(pCLData->PointIndices())->data();
	if(clPointIndices.size() != vertex_sum)
	{
		clPointIndices.resize(vertex_sum);
		for (int i = 0; i < vertex_sum; ++i)
		{
			clPointIndices[i] = i;
		}
		pCLData->SetIndexingDirty(true);
	}

	pCLData->SetStreamDirty(true);

	return true;
}

bool NuiKinfuOpenCLHashScene::Volume2CLMesh(NuiCLMappableData* pCLData)
{
	assert(pCLData);
	if(!pCLData)
		return false;

	return true;
}

bool NuiKinfuOpenCLHashScene::Volume2Mesh(NuiMeshShape* pMesh)
{
	assert(pMesh);
	if(!pMesh)
		return false;

	return true;
}