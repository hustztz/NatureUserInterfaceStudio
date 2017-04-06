#include "NuiKinfuOpenCLHashScene.h"

#include "NuiKinfuOpenCLHashGlobalCache.h"
#include "NuiKinfuOpenCLFrame.h"
#include "NuiKinfuOpenCLAcceleratedFeedbackFrame.h"
#include "NuiKinfuOpenCLCameraState.h"
#include "NuiOpenCLPrefixSum.h"
#include "NuiMarchingCubeTable.h"

#include "Kernels/hashing_gpu_def.h"
#include "Kernels/gpu_def.h"
#include "Foundation/NuiDebugMacro.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"
#include "OpenCLUtilities/NuiOpenCLBufferFactory.h"
#include "OpenCLUtilities/NuiOpenCLFoundationUtils.h"

#include "Shape/NuiCLMappableData.h"

NuiKinfuOpenCLHashScene::NuiKinfuOpenCLHashScene(const NuiHashingSDFConfig& sdfConfig)
	: m_config(sdfConfig)
	, m_pGlobalCache(NULL)
	, m_entriesAllocTypeCL(NULL)
	, m_blockCoordsCL(NULL)
	, m_visibleEntryIDsCL(NULL)
	, m_entriesVisibleTypeCL(NULL)
	, m_numVisibleEntries(0)
	, m_outputIdxCL(NULL)
	, m_MB_edgeTableCL(NULL)
	, m_MB_triTableCL(NULL)
	, m_pScan(NULL)
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
	m_outputIdxCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, sizeof(cl_int), NULL, &err);
	NUI_CHECK_CL_ERR(err);

	m_pScan = new NuiOpenCLPrefixSum(nTotalEntries);

	m_MB_edgeTableCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, 256 * sizeof(int), sEdgeTable, &err);
	NUI_CHECK_CL_ERR(err);
	m_MB_triTableCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, 256 * 16 * sizeof(int), sTriTable, &err);
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
	if (m_outputIdxCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_outputIdxCL);
		NUI_CHECK_CL_ERR(err);
		m_outputIdxCL = NULL;
	}

	SafeDelete(m_pScan);

	if (m_MB_edgeTableCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_MB_edgeTableCL);
		NUI_CHECK_CL_ERR(err);
		m_MB_edgeTableCL = NULL;
	}
	if (m_MB_triTableCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_MB_triTableCL);
		NUI_CHECK_CL_ERR(err);
		m_MB_triTableCL = NULL;
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

	updateGlobalCache(m_config.m_bUseSwapping);

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
	const UINT nTotalEntries = SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE;
	NuiOpenCLFoundationUtils::resetUcharBuffer(m_entriesAllocTypeCL, nTotalEntries);
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
	float oneOverVoxelBlockSize = 1.0f / (m_config.m_virtualVoxelSize * SDF_BLOCK_SIZE);

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(buildHashKernel, idx++, sizeof(cl_mem), &m_entriesVisibleTypeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildHashKernel, idx++, sizeof(cl_mem), &m_entriesAllocTypeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildHashKernel, idx++, sizeof(cl_mem), &m_blockCoordsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildHashKernel, idx++, sizeof(cl_mem), &hashEntriesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildHashKernel, idx++, sizeof(cl_mem), &floatDepthsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildHashKernel, idx++, sizeof(cl_mem), &transformCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildHashKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildHashKernel, idx++, sizeof(cl_float), &m_config.m_truncScale);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(buildHashKernel, idx++, sizeof(cl_float), &oneOverVoxelBlockSize);
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


#ifdef _DEBUG
	cl_uchar* entriesVisibleTypes = new cl_uchar[nTotalEntries];
	err = clEnqueueReadBuffer(
		queue,
		m_entriesVisibleTypeCL,
		CL_TRUE,//blocking
		0, //offset
		nTotalEntries * sizeof(cl_uchar),
		entriesVisibleTypes,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);
	UINT count = 0;
	for (UINT i = 0; i < nTotalEntries; ++i)
	{
		if( entriesVisibleTypes[i] > 0)
			count ++;
	}
	std::cout << count  << std::endl;
	SafeDeleteArray(entriesVisibleTypes);

	cl_int lastFreeBlockId = 0;
	err = clEnqueueReadBuffer(
		queue,
		lastFreeBlockIdCL,
		CL_TRUE,//blocking
		0, //offset
		sizeof(cl_int),
		&lastFreeBlockId,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);
	std::cout << lastFreeBlockId  << std::endl;
#endif
}

UINT	NuiKinfuOpenCLHashScene::BuildVisibleList(cl_mem cameraParamsCL, cl_mem transformCL)
{
	// Get the kernel
	cl_kernel buildVisibleKernel = NuiOpenCLKernelManager::instance().acquireKernel(
			m_pGlobalCache ? E_HASHING_BUILD_VISIBLE_ENLARGED_LIST : E_HASHING_BUILD_VISIBLE_LIST);
	assert(buildVisibleKernel);
	if (!buildVisibleKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_BUILD_HASH_ALLOC' failed!\n");
		return 0;
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

	return m_pScan->prefixSum(m_entriesVisibleTypeCL, m_visibleEntryIDsCL);
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
	m_numVisibleEntries = BuildVisibleList(cameraParamsCL, transformCL);
	if(m_pGlobalCache)
	{
		m_pGlobalCache->reAllocateSwappedOutVoxelBlocks(&m_hashingVoxelData, m_entriesVisibleTypeCL);
	}
}

void	NuiKinfuOpenCLHashScene::IntegrateIntoScene(
	cl_mem floatDepthsCL,
	cl_mem colorsCL,
	cl_mem cameraParamsCL,
	cl_mem transformCL)
{
	if(0 == m_numVisibleEntries)
		return;

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
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_float), &m_config.m_truncScale);
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

#ifdef _DEBUG
	NuiKinfuVoxel* blocks = new NuiKinfuVoxel[SDF_LOCAL_BLOCK_NUM * SDF_BLOCK_SIZE3];
	err = clEnqueueReadBuffer(
		queue,
		voxelBlocksCL,
		CL_TRUE,//blocking
		0, //offset
		SDF_LOCAL_BLOCK_NUM * SDF_BLOCK_SIZE3 * sizeof(NuiKinfuVoxel),
		blocks,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);
	UINT count = 0;
	for (UINT i = 0; i < SDF_LOCAL_BLOCK_NUM * SDF_BLOCK_SIZE3; ++i)
	{
		if( blocks[i].weight > 0 && abs(blocks[i].sdf) < 100)
		{
			count ++;
		}
	}
	std::cout << count  << std::endl;
	SafeDeleteArray(blocks);

	cl_mem lastFreeBlockIdCL = m_hashingVoxelData.getLastFreeBlockIdCL();
	cl_int lastFreeBlockId = 0;
	err = clEnqueueReadBuffer(
		queue,
		lastFreeBlockIdCL,
		CL_TRUE,//blocking
		0, //offset
		sizeof(cl_int),
		&lastFreeBlockId,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);
	std::cout << lastFreeBlockId  << std::endl;
#endif
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

	if(m_pGlobalCache)
	{
		m_pGlobalCache->integrateGlobalIntoLocal(&m_hashingVoxelData, m_config.m_integrationWeightMax);
		m_pGlobalCache->saveToGlobalMemory(&m_hashingVoxelData, m_entriesVisibleTypeCL);
	}

	return true;
}

void	NuiKinfuOpenCLHashScene::updateGlobalCache(bool bEnable)
{
	if(bEnable)
	{
		if(!m_pGlobalCache)
			m_pGlobalCache = new NuiKinfuOpenCLHashGlobalCache();
	}
	else
	{
		SafeDelete(m_pGlobalCache);
	}
}

void	NuiKinfuOpenCLHashScene::CreateExpectedDepths(
	cl_mem expectedRangeCL,
	cl_mem cameraParamsCL,
	cl_mem transformCL)
{
	if(0 == m_numVisibleEntries)
		return;
	if(!expectedRangeCL || !cameraParamsCL || !transformCL)
		return;

	// Get the kernel
	cl_kernel projectBlocksKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_PROJECT_MINMAX_DEPTHS);
	assert(projectBlocksKernel);
	if (!projectBlocksKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_PROJECT_MINMAX_DEPTHS' failed!\n");
		return;
	}

	cl_mem hashEntriesCL = m_hashingVoxelData.getHashEntriesCL();

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(projectBlocksKernel, idx++, sizeof(cl_mem), &expectedRangeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(projectBlocksKernel, idx++, sizeof(cl_mem), &m_visibleEntryIDsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(projectBlocksKernel, idx++, sizeof(cl_mem), &hashEntriesCL);
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

#ifdef _DEBUG
	err = clFinish(queue);
	NUI_CHECK_CL_ERR(err);
#endif
}

void	NuiKinfuOpenCLHashScene::raycast(
	cl_mem renderVerticesCL,
	cl_mem renderNormalsCL,
	cl_mem renderColorsCL,
	cl_mem expectedRangeCL,
	cl_mem cameraParamsCL,
	cl_mem transformCL,
	UINT nWidth, UINT nHeight
	)
{
	/*if(0 == m_numVisibleEntries)
		return;*/

	if(!renderVerticesCL || !renderNormalsCL || !cameraParamsCL || !transformCL)
		return;

	// Get the kernel
	cl_kernel raycastKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_RAYCAST_RENDER);
	assert(raycastKernel);
	if (!raycastKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_RAYCAST_RENDER' failed!\n");
		return;
	}

	cl_mem hashEntriesCL = m_hashingVoxelData.getHashEntriesCL();
	cl_mem voxelBlocksCL = m_hashingVoxelData.getVoxelBlocksCL();
	const float oneOverVoxelSize = 1.0f / m_config.m_virtualVoxelSize;

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &renderVerticesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &renderNormalsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &renderColorsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &expectedRangeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &hashEntriesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &voxelBlocksCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &transformCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_float), &m_config.m_virtualVoxelSize);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_float), &oneOverVoxelSize);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_float), &m_config.m_truncation);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	size_t kernelGlobalSize[2] = { nWidth, nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		raycastKernel,
		2,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

#ifdef _DEBUG
	err = clFinish(queue);
	NUI_CHECK_CL_ERR(err);
#endif
}

void	NuiKinfuOpenCLHashScene::forwardRender(
	cl_mem renderVerticesCL,
	cl_mem renderNormalsCL,
	cl_mem renderColorsCL,
	cl_mem expectedRangeCL,
	cl_mem cameraParamsCL,
	cl_mem transformCL,
	UINT nWidth, UINT nHeight
	)
{
	/*if(0 == m_numVisibleEntries)
		return;*/

	if(!renderVerticesCL || !renderNormalsCL || !cameraParamsCL || !transformCL)
		return;

	// Get the kernel
	cl_kernel forwardKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_FORWARD_PROJECT_RENDER);
	assert(forwardKernel);
	if (!forwardKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_FORWARD_PROJECT_RENDER' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(forwardKernel, idx++, sizeof(cl_mem), &renderVerticesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(forwardKernel, idx++, sizeof(cl_mem), &renderNormalsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(forwardKernel, idx++, sizeof(cl_mem), &renderColorsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(forwardKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(forwardKernel, idx++, sizeof(cl_mem), &transformCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(forwardKernel, idx++, sizeof(cl_float), &m_config.m_virtualVoxelSize);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	size_t kernelGlobalSize[2] = { nWidth, nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		forwardKernel,
		2,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

#ifdef _DEBUG
	err = clFinish(queue);
	NUI_CHECK_CL_ERR(err);
#endif
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
		return;
	cl_mem transformCL = pCLCamera->GetCameraTransformBuffer();
	cl_mem cameraParamsCL = pCLCamera->GetCameraParamsBuffer();
	if(!cameraParamsCL || !transformCL)
		return;

	if(!pFeedbackFrame)
		return;
	NuiKinfuOpenCLFeedbackFrame* pCLFeedbackFrame = dynamic_cast<NuiKinfuOpenCLFeedbackFrame*>(pFeedbackFrame);
	if(!pCLFeedbackFrame)
		return;

	NuiKinfuOpenCLAcceleratedFeedbackFrame* pCLAcceleratedFeedbackFrame = dynamic_cast<NuiKinfuOpenCLAcceleratedFeedbackFrame*>(pFeedbackFrame);
	if(pCLAcceleratedFeedbackFrame)
	{
		pCLAcceleratedFeedbackFrame->resetExpectedRange();
		CreateExpectedDepths(pCLAcceleratedFeedbackFrame->getExpectedRangeCL(), cameraParamsCL, transformCL);
	}

	raycast(
		pCLFeedbackFrame->GetVertexBuffer(),
		pCLFeedbackFrame->GetNormalBuffer(),
		pCLFeedbackFrame->GetColorBuffer(),
		pCLAcceleratedFeedbackFrame ? pCLAcceleratedFeedbackFrame->getExpectedRangeCL() : NULL,
		cameraParamsCL, transformCL,
		pCLFeedbackFrame->GetWidth(), pCLFeedbackFrame->GetHeight()
		);
}

bool NuiKinfuOpenCLHashScene::Volume2CLVertices(NuiCLMappableData* pCLData)
{
	assert(pCLData);
	if(!pCLData)
		return false;

	if(!m_dirty)
		return false;
	m_dirty = false;

	if(0 == m_numVisibleEntries)
		return false;

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
		NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_FETCH_SCENE);
	assert(fetchKernel);
	if (!fetchKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_FETCH_SCENE' failed!\n");
		return false;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	cl_int vertex_id = 0;
	err = clEnqueueWriteBuffer(
		queue,
		m_outputIdxCL,
		CL_FALSE,//blocking
		0,
		sizeof(cl_int),
		&vertex_id,
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

	cl_mem hashEntriesCL = m_hashingVoxelData.getHashEntriesCL();
	cl_mem voxelBlocksCL = m_hashingVoxelData.getVoxelBlocksCL();

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
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &positionsGL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &colorsGL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &m_outputIdxCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &m_visibleEntryIDsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &hashEntriesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &voxelBlocksCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_float), &m_config.m_virtualVoxelSize);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_float), &m_config.m_truncScale);
	NUI_CHECK_CL_ERR(err);

#ifdef _GPU_PROFILER
	cl_event timing_event;
	cl_ulong time_start, time_end;
#endif
	// Run kernel to calculate 
	//size_t local_ws[1] = {SDF_BLOCK_SIZE3};
	size_t kernelGlobalSize[1] = { m_numVisibleEntries  };
	err = clEnqueueNDRangeKernel(
		queue,
		fetchKernel,
		1,
		nullptr,
		kernelGlobalSize,
		nullptr,
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
		m_outputIdxCL,
		CL_TRUE,//blocking
		0,
		sizeof(cl_int),
		&vertex_id,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	if(vertex_id <= 0)
		return false;

	NuiMappableAccessor::asVectorImpl(pCLData->TriangleIndices())->data().clear();
	NuiMappableAccessor::asVectorImpl(pCLData->WireframeIndices())->data().clear();

	std::vector<unsigned int>& clPointIndices =
		NuiMappableAccessor::asVectorImpl(pCLData->PointIndices())->data();
	if(clPointIndices.size() != vertex_id)
	{
		clPointIndices.resize(vertex_id);
		for (int i = 0; i < vertex_id; ++i)
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

	if(0 == m_numVisibleEntries || !m_outputIdxCL)
		return false;

	if(!m_dirty)
		return true;
	clearDirty();

	cl_kernel marchingCubeKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_MARCHING_CUBE);
	assert(marchingCubeKernel);
	if (!marchingCubeKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_MARCHING_CUBE' failed!\n");
		return false;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	cl_int vertex_id = 0;
	err = clEnqueueWriteBuffer(
		queue,
		m_outputIdxCL,
		CL_FALSE,//blocking
		0,
		sizeof(cl_int),
		&vertex_id,
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

	cl_mem hashEntriesCL = m_hashingVoxelData.getHashEntriesCL();
	cl_mem voxelBlocksCL = m_hashingVoxelData.getVoxelBlocksCL();

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
	err = clSetKernelArg(marchingCubeKernel, idx++, sizeof(cl_mem), &positionsGL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(marchingCubeKernel, idx++, sizeof(cl_mem), &colorsGL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(marchingCubeKernel, idx++, sizeof(cl_mem), &m_outputIdxCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(marchingCubeKernel, idx++, sizeof(cl_mem), &m_visibleEntryIDsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(marchingCubeKernel, idx++, sizeof(cl_mem), &hashEntriesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(marchingCubeKernel, idx++, sizeof(cl_mem), &voxelBlocksCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(marchingCubeKernel, idx++, sizeof(cl_mem), &m_MB_edgeTableCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(marchingCubeKernel, idx++, sizeof(cl_mem), &m_MB_triTableCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(marchingCubeKernel, idx++, sizeof(cl_float), &m_config.m_virtualVoxelSize);
	NUI_CHECK_CL_ERR(err);
	

#ifdef _GPU_PROFILER
	cl_event timing_event;
	cl_ulong time_start, time_end;
#endif
	// Run kernel to calculate
	size_t kernelGlobalSize[1] = { m_numVisibleEntries };
	err = clEnqueueNDRangeKernel(
		queue,
		marchingCubeKernel,
		1,
		nullptr,
		kernelGlobalSize,
		nullptr,
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

	err = clEnqueueReadBuffer(
		queue,
		m_outputIdxCL,
		CL_TRUE,//blocking
		0,
		sizeof(cl_int),
		&vertex_id,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

#ifdef _GPU_PROFILER
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_START, sizeof(time_start), &time_start, NULL);
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_END, sizeof(time_end), &time_end, NULL);
	std::cout << "marching cube:" << (time_end - time_start) << std::endl;
	clReleaseEvent(timing_event);
#endif

	if(vertex_id <= 0)
		return false;

	NuiMappableAccessor::asVectorImpl(pCLData->PointIndices())->data().clear();

	if(pCLData->TriangleIndices().size() != MAX_OUTPUT_VERTEX_SIZE)
	{
		std::vector<unsigned int>& clTriangleIndices =
			NuiMappableAccessor::asVectorImpl(pCLData->TriangleIndices())->data();
		std::vector<unsigned int>& clWireframeIndices =
			NuiMappableAccessor::asVectorImpl(pCLData->WireframeIndices())->data();

		clTriangleIndices.resize(MAX_OUTPUT_VERTEX_SIZE);
		clWireframeIndices.resize(MAX_OUTPUT_VERTEX_SIZE*2);
		for (int i = 0; i < MAX_OUTPUT_VERTEX_SIZE; ++i)
		{
			clTriangleIndices[i] = i;
			if(i % 3 == 2)
			{
				clWireframeIndices[2*i  ] = i;
				clWireframeIndices[2*i+1] = i - 2;
			}
			else
			{
				clWireframeIndices[2*i  ] = i;
				clWireframeIndices[2*i+1] = i + 1;
			}
		}
		pCLData->SetIndexingDirty(true);
	}

	// Set bounding box
	//pCLData->SetBoundingBox(SgVec3f(-m_config.dimensions[0]/2, -m_config.dimensions[1]/2, -m_config.dimensions[2]/2), SgVec3f(m_config.dimensions[0]/2, m_config.dimensions[1]/2, m_config.dimensions[2]/2));

	pCLData->SetStreamDirty(true);

	return true;
}

bool NuiKinfuOpenCLHashScene::Volume2Mesh(NuiMeshShape* pMesh)
{
	assert(pMesh);
	if(!pMesh)
		return false;

	return true;
}