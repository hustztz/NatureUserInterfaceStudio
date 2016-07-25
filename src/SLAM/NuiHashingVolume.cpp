#include "NuiHashingVolume.h"

#include "NuiHashingSDF.h"
#include "NuiHashingChunkGrid.h"
#include "NuiKinfuTransform.h"

#include "Foundation/NuiMatrixUtilities.h"
#include "Foundation/NuiDebugMacro.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"

#include "Shape/NuiCLMappableData.h"

NuiHashingVolume::NuiHashingVolume(const NuiHashingSDFConfig& sdfConfig)
	: m_pSDFData(NULL)
	, m_pChunkGrid(NULL)
{
	m_pSDFData = new NuiHashingSDF(sdfConfig);
	//m_pChunkGrid = new NuiHashingChunkGrid(&m_sdfData);

	reset();
}

NuiHashingVolume::~NuiHashingVolume()
{
	SafeDelete(m_pSDFData);
	SafeDelete(m_pChunkGrid);
}

void	NuiHashingVolume::reset()
{
	m_pSDFData->reset();
	if(m_pChunkGrid)
		m_pChunkGrid->reset();

	NuiKinfuVolume::reset();
}

void NuiHashingVolume::raycastRender(
	NuiHashingSDF* pSDF,
	cl_mem cameraParamsCL,
	cl_mem transformCL,
	cl_mem verticesCL,
	cl_mem normalsCL,
	float rayIncrement,
	float thresSampleDist,
	float thresDist,
	float minDepth,
	float maxDepth,
	UINT nWidth, UINT nHeight
	)
{
	if(!pSDF)
		return;

	// Get the kernel
	cl_kernel raycastKernel = nullptr;
	NuiOpenCLKernelManager::instance().acquireKernel(E_HASHING_RAYCAST_SDF);
	assert(raycastKernel);
	if (!raycastKernel)
	{
		NUI_ERROR("Get kernel 'E_HASHING_RAYCAST_SDF' failed!\n");
		return;
	}

	cl_mem hashCL = pSDF->getHashCL();
	cl_mem SDFBlocksCL = pSDF->getSDFBlocksCL();
	const NuiHashingSDFConfig& hashParams = pSDF->getConfig();

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &transformCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &verticesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &normalsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), NULL);
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
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_uint), &thresDist);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_uint), &rayIncrement);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_uint), &minDepth);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_uint), &maxDepth);
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

void	NuiHashingVolume::incrementVolume(
	cl_mem floatDepthsCL,
	cl_mem colorsCL,
	cl_mem normalsCL,
	cl_mem cameraParamsCL,
	const NuiKinfuTransform& currPos,
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
		currPos.getTransformCL(),
		m_pChunkGrid ? m_pChunkGrid->getBitMaskCL() : NULL,
		streamingVoxelExtends,
		streamingGridDimensions,
		streamingMinGridPos
		);
	m_lastIntegrationRotation = currPos.getRotation();
	m_lastIntegrationTranslation = currPos.getTranslation();
	setDirty();
}

bool	NuiHashingVolume::evaluateVolume(
	cl_mem floatDepthsCL,
	cl_mem colorsCL,
	cl_mem normalsCL,
	cl_mem renderVertices,
	cl_mem renderNormals,
	cl_mem cameraParamsCL,
	const NuiKinfuTransform& currPos,
	UINT nWidth, UINT nHeight
	)
{
	///////////////////////////////////////////////////////////////////////////////////////////
	// Integration check - We do not integrate volume if camera does not move.  
	float rnorm = NuiMatrixUtilities::rodrigues2(currPos.getRotation().inverse() * m_lastIntegrationRotation).norm();
	float tnorm = (currPos.getTranslation() - m_lastIntegrationTranslation).norm();
	const float alpha = 1.f;
	bool integrate = (rnorm + alpha * tnorm)/2 >= m_integration_metric_threshold;
	// Integrate
	if (integrate)
	{
		incrementVolume(floatDepthsCL, colorsCL, normalsCL, cameraParamsCL, currPos, nWidth, nHeight);
	}
	raycastRender(
		m_pSDFData,
		cameraParamsCL,
		currPos.getTransformCL(),
		renderVertices,
		renderNormals,
		m_rayIncrement,
		m_thresSampleDist,
		m_thresDist,
		m_minDepth,
		m_maxDepth,
		nWidth, nHeight);
	return integrate;
}

bool NuiHashingVolume::Volume2CLVertices(NuiCLMappableData* pCLData)
{
	assert(pCLData);
	if(!pCLData)
		return false;

	if(!m_dirty)
		return false;

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

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	cl_int mutex = 0;
	err = clEnqueueWriteBuffer(
		queue,
		m_mutexCL,
		CL_FALSE,//blocking
		0,
		sizeof(cl_int),
		&mutex,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

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
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &m_volumeOutputVerticesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &m_volumeOutputColorsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_int), &m_max_output_vertex_size);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &m_mutexCL);
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

	//Push the cached point cloud
	m_cachedPointCloud.readLock();
	const int cachedVertexSize = m_cachedPointCloud.pointSize();
	const int point_count = cachedVertexSize + vertex_sum;
	if(positions.size() != point_count)
		positions.resize(point_count);
	memcpy((void*)(positions.data()+vertex_sum), m_cachedPointCloud.getVertices(), cachedVertexSize*sizeof(SgVec3f));
	if(colors.size() != point_count)
		colors.resize(point_count);
	memcpy((void*)(colors.data()+vertex_sum), m_cachedPointCloud.getColors(), cachedVertexSize*sizeof(SgVec4f));
	m_cachedPointCloud.readUnlock();

	std::vector<unsigned int>& clTriangleIndices =
		NuiMappableAccessor::asVectorImpl(pCLData->TriangleIndices())->data();
	clTriangleIndices.clear();

	std::vector<unsigned int>& clWireframeIndices =
		NuiMappableAccessor::asVectorImpl(pCLData->WireframeIndices())->data();
	clWireframeIndices.clear();

	std::vector<unsigned int>& clPointIndices =
		NuiMappableAccessor::asVectorImpl(pCLData->PointIndices())->data();
	if(clPointIndices.size() != point_count)
	{
		clPointIndices.resize(point_count);
		for (int i = 0; i < point_count; ++i)
		{
			clPointIndices[i] = i;
		}
		pCLData->SetIndexingDirty(true);
	}

	pCLData->SetStreamDirty(true);
	m_dirty = false;

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