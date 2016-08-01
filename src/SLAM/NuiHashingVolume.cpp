#include "NuiHashingVolume.h"

#include "NuiHashingSDF.h"
#include "NuiHashingChunkGrid.h"
#include "NuiKinfuTransform.h"

#include "Foundation/NuiDebugMacro.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"

#include "Shape/NuiCLMappableData.h"

NuiHashingVolume::NuiHashingVolume(const NuiHashingSDFConfig& sdfConfig, const NuiHashingRaycastConfig& raycastConfig)
	: m_pSDFData(NULL)
	, m_pChunkGrid(NULL)
	, m_raycastConfig(raycastConfig)
{
	m_pSDFData = new NuiHashingSDF(sdfConfig);

	reset();
}

NuiHashingVolume::~NuiHashingVolume()
{
	SafeDelete(m_pSDFData);
	SafeDelete(m_pChunkGrid);
}

bool NuiHashingVolume::log(const std::string& fileName) const
{
	return true;
}

void	NuiHashingVolume::reset()
{
	m_pSDFData->reset();
	if(m_pChunkGrid)
		m_pChunkGrid->reset();

	setDirty();
}

void	NuiHashingVolume::updateChunkGridConfig(const NuiHashingChunkGridConfig& chunkGridConfig)
{
	if(chunkGridConfig.m_enable)
	{
		if(!m_pChunkGrid)
			m_pChunkGrid = new NuiHashingChunkGrid(m_pSDFData);
		m_pChunkGrid->updateConfig(chunkGridConfig);
	}
	else
	{
		SafeDelete(m_pChunkGrid);
	}
}

void NuiHashingVolume::raycastRender(
	cl_mem renderVerticesCL,
	cl_mem renderNormalsCL,
	cl_mem renderColorsCL,
	cl_mem cameraParamsCL,
	cl_mem transformCL,
	UINT nWidth, UINT nHeight
	)
{
	if(!m_pSDFData)
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

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

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
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &renderColorsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &hashCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &SDFBlocksCL);
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
}

void	NuiHashingVolume::integrateVolume(
	cl_mem floatDepthsCL,
	cl_mem normalsCL,
	cl_mem colorsCL,
	cl_mem cameraParamsCL,
	cl_mem transformCL,
	UINT nWidth, UINT nHeight
	)
{
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
	m_pSDFData->integrate(
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
	setDirty();
}

bool NuiHashingVolume::Volume2CLVertices(NuiCLMappableData* pCLData)
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
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &m_volumeOutputVerticesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &m_volumeOutputColorsCL);
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

	std::vector<SgVec3f>& positions = NuiMappableAccessor::asVectorImpl(pCLData->PositionStream())->data();
	if(positions.size() != vertex_sum)
		positions.resize(vertex_sum);

	clEnqueueReadBuffer(
		queue,
		m_volumeOutputVerticesCL,
		CL_FALSE,//blocking
		0,
		vertex_sum * 3 * sizeof(float),
		positions.data(),
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	std::vector<SgVec4f>& colors = NuiMappableAccessor::asVectorImpl(pCLData->ColorStream())->data();
	if(colors.size() != vertex_sum)
		colors.resize(vertex_sum);

	clEnqueueReadBuffer(
		queue,
		m_volumeOutputColorsCL,
		CL_FALSE,//blocking
		0,
		vertex_sum * 4 * sizeof(float),
		colors.data(),
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	std::vector<unsigned int>& clTriangleIndices =
		NuiMappableAccessor::asVectorImpl(pCLData->TriangleIndices())->data();
	clTriangleIndices.clear();

	std::vector<unsigned int>& clWireframeIndices =
		NuiMappableAccessor::asVectorImpl(pCLData->WireframeIndices())->data();
	clWireframeIndices.clear();

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

bool NuiHashingVolume::Volume2CLMesh(NuiCLMappableData* pCLData)
{
	assert(pCLData);
	if(!pCLData)
		return false;

	return true;
}

bool NuiHashingVolume::Volume2Mesh(NuiMeshShape* pMesh)
{
	assert(pMesh);
	if(!pMesh)
		return false;

	return true;
}