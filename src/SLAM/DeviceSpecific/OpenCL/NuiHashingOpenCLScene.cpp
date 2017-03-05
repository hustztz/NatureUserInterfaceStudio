#include "NuiHashingOpenCLScene.h"

#include "NuiKinfuOpenCLFrame.h"
#include "NuiKinfuOpenCLFeedbackFrame.h"
#include "NuiKinfuOpenCLCameraState.h"
#include "NuiHashingOpenCLSDF.h"
#include "NuiHashingOpenCLChunkGrid.h"

#include "../NuiKinfuCameraState.h"

#include "Kernels/gpu_def.h"
#include "Foundation/NuiDebugMacro.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"
#include "OpenCLUtilities/NuiOpenCLBufferFactory.h"
#include "OpenCLUtilities/NuiOfflineRenderFactory.h"

#include "Shape/NuiCLMappableData.h"

NuiHashingOpenCLScene::NuiHashingOpenCLScene(const NuiHashingSDFConfig& sdfConfig, const NuiHashingRaycastConfig& raycastConfig)
	: m_pSDFData(NULL)
	, m_pChunkGrid(NULL)
	, m_vertexSumCL(NULL)
	, m_numOccupiedBlocks(0)
	, m_raycastConfig(raycastConfig)
	, m_raycastVertexBuffer("raycastVertex")
	, m_offlineRenderDirty(false)
{
	m_pSDFData = new NuiHashingOpenCLSDF(sdfConfig);

	NuiOfflineRenderFactory::initializeOfflineRender();
	NuiMappableAccessor::asVectorImpl(m_raycastVertexBuffer)->data().resize(sdfConfig.m_numSDFBlocks * 6);
	NuiOpenCLBufferFactory::asColor4fBufferCL(m_raycastVertexBuffer);
	NuiTextureMappableAccessor::updateImpl(
		m_rayIntervalMinBuffer,
		512,
		424,
		NULL
		);
	NuiOpenCLBufferFactory::asFrameTexture2DCL(m_rayIntervalMinBuffer);
	NuiTextureMappableAccessor::updateImpl(
		m_rayIntervalMaxBuffer,
		512,
		424,
		NULL
		);
	NuiOpenCLBufferFactory::asFrameTexture2DCL(m_rayIntervalMaxBuffer);

	reset();
	AcquireBuffer();
}

NuiHashingOpenCLScene::~NuiHashingOpenCLScene()
{
	SafeDelete(m_pSDFData);
	SafeDelete(m_pChunkGrid);

	NuiMappableAccessor::reset(m_raycastVertexBuffer);
	ReleaseBuffer();
}

void NuiHashingOpenCLScene::AcquireBuffer()
{
	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();

	m_vertexSumCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, sizeof(cl_int), NULL, &err);
	NUI_CHECK_CL_ERR(err);
}


void NuiHashingOpenCLScene::ReleaseBuffer()
{
	if (m_vertexSumCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_vertexSumCL);
		NUI_CHECK_CL_ERR(err);
		m_vertexSumCL = NULL;
	}
}

bool NuiHashingOpenCLScene::log(const std::string& fileName) const
{
	return true;
}

void	NuiHashingOpenCLScene::reset()
{
	m_pSDFData->reset();
	if(m_pChunkGrid)
		m_pChunkGrid->reset();
	m_numOccupiedBlocks = 0;

	setDirty();
	m_offlineRenderDirty = false;
}

void	NuiHashingOpenCLScene::updateChunkGridConfig(const NuiHashingChunkGridConfig& chunkGridConfig)
{
	if(chunkGridConfig.m_enable)
	{
		if(!m_pChunkGrid)
			m_pChunkGrid = new NuiHashingOpenCLChunkGrid(m_pSDFData);
		m_pChunkGrid->updateConfig(chunkGridConfig);
	}
	else
	{
		SafeDelete(m_pChunkGrid);
	}
}

void NuiHashingOpenCLScene::rayIntervalSplatting(cl_mem cameraParamsCL, cl_mem transformCL)
{
	if(!m_pSDFData || !m_numOccupiedBlocks)
		return;

	if(!cameraParamsCL || !transformCL)
		return;

	// Get the kernel
	cl_kernel intervalSplatKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_RAY_INTERVAL_SPLAT);
	assert(intervalSplatKernel);
	if (!intervalSplatKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_RAY_INTERVAL_SPLAT' failed!\n");
		return;
	}

	cl_mem hashCompactifiedCL = m_pSDFData->getHashCompactifiedCL();
	const NuiHashingSDFConfig& hashParams = m_pSDFData->getConfig();
	cl_mem raycastVertexBufferGL = NuiOpenCLBufferFactory::asColor4fBufferCL(m_raycastVertexBuffer);

	// Acquire OpenGL objects before use
	cl_mem glObjs[] = {
		raycastVertexBufferGL
	};

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	//glFinish();
	err = clEnqueueAcquireGLObjects(queue, sizeof(glObjs) / sizeof(cl_mem), glObjs,
		0, nullptr, nullptr);
	NUI_CHECK_CL_ERR(err);
	/*openclutil::enqueueAcquireHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);*/

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(intervalSplatKernel, idx++, sizeof(cl_mem), &raycastVertexBufferGL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(intervalSplatKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(intervalSplatKernel, idx++, sizeof(cl_mem), &transformCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(intervalSplatKernel, idx++, sizeof(cl_mem), &hashCompactifiedCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(intervalSplatKernel, idx++, sizeof(cl_float), &hashParams.m_virtualVoxelSize);
	NUI_CHECK_CL_ERR(err);

	m_numOccupiedBlocks = std::min(m_numOccupiedBlocks, (UINT)m_raycastVertexBuffer.size()/6);
	// Run kernel to calculate
	size_t kernelGlobalSize[1] = { m_numOccupiedBlocks };
	err = clEnqueueNDRangeKernel(
		queue,
		intervalSplatKernel,
		1,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	err = clFinish(queue);
	NUI_CHECK_CL_ERR(err);

	// Release OpenGL objects
	/*openclutil::enqueueReleaseHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);*/

	err = clEnqueueReleaseGLObjects(queue, sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);
	NUI_CHECK_CL_ERR(err);
}

void	NuiHashingOpenCLScene::raycast(
	cl_mem renderVerticesCL,
	cl_mem renderNormalsCL,
	cl_mem renderIntensitiesCL,
	cl_mem cameraParamsCL,
	cl_mem transformCL,
	UINT nWidth, UINT nHeight
	)
{
	if(!m_pSDFData || !m_numOccupiedBlocks)
		return;

	if(!renderVerticesCL || !renderNormalsCL || !cameraParamsCL || !transformCL)
		return;

	// Get the kernel
	cl_kernel raycastKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_RAYCAST_SDF);
	assert(raycastKernel);
	if (!raycastKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_RAYCAST_SDF' failed!\n");
		return;
	}

	cl_mem hashCL = m_pSDFData->getHashCL();
	cl_mem SDFBlocksCL = m_pSDFData->getSDFBlocksCL();
	const NuiHashingSDFConfig& hashParams = m_pSDFData->getConfig();

	float rayIncrement = m_raycastConfig.m_rayIncrementFactor * m_pSDFData->getConfig().m_truncation;
	float thresSampleDist = m_raycastConfig.m_thresSampleDistFactor * rayIncrement;
	float thresDist = m_raycastConfig.m_thresDistFactor * rayIncrement;

	cl_mem rayIntervalMinGL = NuiOpenCLBufferFactory::asFrameTexture2DCL(m_rayIntervalMinBuffer);
	cl_mem rayIntervalMaxGL = NuiOpenCLBufferFactory::asFrameTexture2DCL(m_rayIntervalMaxBuffer);

	// Acquire OpenGL objects before use
	cl_mem glObjs[] = {
		rayIntervalMinGL,
		rayIntervalMaxGL
	};

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	//glFinish();
	err = clEnqueueAcquireGLObjects(queue, sizeof(glObjs) / sizeof(cl_mem), glObjs,
		0, nullptr, nullptr);
	NUI_CHECK_CL_ERR(err);
	/*openclutil::enqueueAcquireHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);*/

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &transformCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &renderVerticesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &renderNormalsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &renderIntensitiesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &hashCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &SDFBlocksCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &rayIntervalMinGL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &rayIntervalMaxGL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_float), &hashParams.m_virtualVoxelSize);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_float), &hashParams.m_hashNumBuckets);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_uint), &hashParams.m_hashMaxCollisionLinkedListSize);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_float), &thresSampleDist);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_float), &thresDist);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_float), &rayIncrement);
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

	err = clFinish(queue);
	NUI_CHECK_CL_ERR(err);

	// Release OpenGL objects
	/*openclutil::enqueueReleaseHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);*/

	err = clEnqueueReleaseGLObjects(queue, sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);
	NUI_CHECK_CL_ERR(err);
}

void	NuiHashingOpenCLScene::offlineRender()
{
	if(m_offlineRenderDirty)
	{
		rayIntervalSplatting(m_cameraParamsCL, m_transformCL);
		NuiOfflineRenderFactory::runOfflineRender(m_raycastVertexBuffer, m_rayIntervalMinBuffer, m_rayIntervalMaxBuffer, (int)m_numOccupiedBlocks, m_sensorDepthMin, m_sensorDepthMax);
		raycast(m_renderVerticesCL, m_renderNormalsCL, m_renderIntensitiesCL, m_cameraParamsCL, m_transformCL, m_nWidth, m_nHeight);
		
		m_offlineRenderDirty = false;
	}
}

void NuiHashingOpenCLScene::raycastRender(
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

	m_offlineRenderDirty = true;
	do 
	{
	} while (m_offlineRenderDirty);

}

bool	NuiHashingOpenCLScene::integrateVolume(
	NuiKinfuFrame*			pFrame,
	NuiKinfuFeedbackFrame*	pFeedbackFrame,
	NuiKinfuCameraState*	pCameraState)
{
	if(!pCameraState)
		return false;
	NuiKinfuOpenCLCameraState* pCLCamera = dynamic_cast<NuiKinfuOpenCLCameraState*>(pCameraState);
	if(!pCLCamera)
		return false;
	cl_mem transformCL = pCLCamera->GetCameraTransformBuffer();
	if(!transformCL)
		return false;

	if(!pFrame)
		return false;
	NuiKinfuOpenCLFrame* pCLFrame = dynamic_cast<NuiKinfuOpenCLFrame*>(pFrame);
	if(!pCLFrame)
		return false;

	if(!pFeedbackFrame)
		return false;
	NuiKinfuOpenCLFeedbackFrame* pCLFeedbackFrame = dynamic_cast<NuiKinfuOpenCLFeedbackFrame*>(pFeedbackFrame);
	if(!pCLFeedbackFrame)
		return false;

	cl_mem floatDepthsCL = pCLFrame->GetDepthBuffer();
	cl_mem cameraParamsCL = pCLCamera->GetCameraParamsBuffer();
	if(!floatDepthsCL || !cameraParamsCL || !transformCL)
		return false;

	cl_mem normalsCL = pCLFeedbackFrame->GetNormalBuffer();
	cl_mem colorsCL = pCLFrame->GetColorBuffer();
	UINT nWidth = pCLFrame->GetWidth();
	UINT nHeight = pCLFrame->GetHeight();

	SgVec3f streamingVoxelExtends;
	SgVec3i	streamingGridDimensions;
	SgVec3i	streamingMinGridPos;
	if(m_pChunkGrid)
	{
		const NuiHashingChunkGridConfig& streamingConfig = m_pChunkGrid->getConfig();
		streamingVoxelExtends = streamingConfig.m_voxelExtends;
		streamingGridDimensions = streamingConfig.m_gridDimensions;
		streamingMinGridPos = streamingConfig.m_minGridPos;
	}
	m_numOccupiedBlocks = m_pSDFData->integrate(
		nWidth, nHeight,
		floatDepthsCL,
		colorsCL,
		cameraParamsCL,
		transformCL,
		m_pChunkGrid ? m_pChunkGrid->getBitMaskCL() : NULL,
		streamingVoxelExtends,
		streamingGridDimensions,
		streamingMinGridPos
		);
	if(m_numOccupiedBlocks)
		setDirty();

	return true;
}

bool NuiHashingOpenCLScene::Volume2CLVertices(NuiCLMappableData* pCLData)
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

bool NuiHashingOpenCLScene::Volume2CLMesh(NuiCLMappableData* pCLData)
{
	assert(pCLData);
	if(!pCLData)
		return false;

	return true;
}

bool NuiHashingOpenCLScene::Volume2Mesh(NuiMeshShape* pMesh)
{
	assert(pMesh);
	if(!pMesh)
		return false;

	return true;
}