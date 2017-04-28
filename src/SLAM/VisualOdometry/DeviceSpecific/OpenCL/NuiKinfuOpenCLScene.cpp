#include "NuiKinfuOpenCLScene.h"

#include "NuiKinfuOpenCLFrame.h"
#include "NuiKinfuOpenCLFeedbackFrame.h"
#include "NuiKinfuOpenCLCameraState.h"

#include "../NuiKinfuCameraState.h"
#include "Foundation/NuiDebugMacro.h"
#include "Foundation/NuiTimeLog.h"
#include "NuiMarchingCubeTable.h"
#include "Shape/NuiCLMappableData.h"
#include "Shape\NuiMeshShape.h"

#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"
#include "OpenCLUtilities/NuiOpenCLBufferFactory.h"

static const std::string sVolume2Vertices("Volume2Vertices");

NuiKinfuOpenCLScene::NuiKinfuOpenCLScene(const NuiKinfuVolumeConfig& config)
	: m_config(config)
	, m_vertexSumCL(NULL)
	, m_volumeCL(NULL)
	, m_colorVolumeCL(NULL)
	, m_volume_paramsCL(NULL)
	, m_MB_numVertsTableCL(NULL)
	, m_MB_triTableCL(NULL)
{
	m_tsdf_params.resolution[0] = (float)m_config.resolution(0);
	m_tsdf_params.resolution[1] = (float)m_config.resolution(1);
	m_tsdf_params.resolution[2] = (float)m_config.resolution(2);
	m_tsdf_params.dimension[0] = m_config.dimensions(0);
	m_tsdf_params.dimension[1] = m_config.dimensions(1);
	m_tsdf_params.dimension[2] = m_config.dimensions(2);
	m_tsdf_params.cell_size[0] = m_config.dimensions(0)/m_config.resolution(0);
	m_tsdf_params.cell_size[1] = m_config.dimensions(1)/m_config.resolution(1);
	m_tsdf_params.cell_size[2] = m_config.dimensions(2)/m_config.resolution(2);
	m_tsdf_params.tranc_dist = std::max (m_config.tranc_dist, 2.1f * std::max (m_tsdf_params.cell_size[0], std::max (m_tsdf_params.cell_size[1], m_tsdf_params.cell_size[2])));

	AcquireBuffer(m_config.bHas_color_volume);
	reset();
}

NuiKinfuOpenCLScene::~NuiKinfuOpenCLScene()
{
	ReleaseBuffer();
}

const Vector3f& NuiKinfuOpenCLScene::getDimensions() const
{
    return m_config.dimensions;
}

const Vector3i& NuiKinfuOpenCLScene::getResolution() const
{
	return m_config.resolution;
}

const Vector3f NuiKinfuOpenCLScene::getVoxelSize() const
{    
	return m_config.dimensions.array () / m_config.resolution.array().cast<float>();
}

float NuiKinfuOpenCLScene::getTsdfTruncDist () const
{
	return m_tsdf_params.tranc_dist;
}

bool NuiKinfuOpenCLScene::log(const std::string& fileName) const
{
	return m_config.log(fileName);
}

SgVec3f NuiKinfuOpenCLScene::getNodeCoo(int x, int y, int z)
{
	return SgVec3f(
		((float)x + 0.5f - m_config.resolution[0] / 2.0f) * m_tsdf_params.cell_size[0],
		((float)(-y) - 0.5f + m_config.resolution[1] / 2.0f) * m_tsdf_params.cell_size[1],
		((float)z + 0.5f - m_config.resolution[2] / 2.0f) * m_tsdf_params.cell_size[2]
	);
}

void NuiKinfuOpenCLScene::reset()
{
	// Get the kernel
	cl_kernel initializeKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_INITIALIZE_VOLUME);
	assert(initializeKernel);
	if (!initializeKernel)
	{
		NUI_ERROR("Get kernel 'E_INITIALIZE_VOLUME' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	Vector3i voxel_offset(0, 0, 0);
	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(initializeKernel, idx++, sizeof(cl_mem), &m_volumeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(initializeKernel, idx++, sizeof(cl_mem), &m_colorVolumeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(initializeKernel, idx++, sizeof(cl_mem), &m_volume_paramsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(initializeKernel, idx++, sizeof(cl_int3), voxel_offset.data());
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	size_t kernelGlobalSize[3] = { m_config.resolution(0), m_config.resolution(1), m_config.resolution(2) };
	err = clEnqueueNDRangeKernel(
		queue,
		initializeKernel,
		3,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	m_cachedPointCloud.clear();
	setDirty();
}


void NuiKinfuOpenCLScene::AcquireBuffer(bool bHas_color_volume)
{
	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();
	
	m_vertexSumCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, sizeof(cl_int), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	m_volumeCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_config.resolution[0]*m_config.resolution[1]*m_config.resolution[2]*sizeof(cl_short2), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	if(bHas_color_volume)
	{
		m_colorVolumeCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, m_config.resolution[0]*m_config.resolution[1]*m_config.resolution[2]*sizeof(cl_char4), NULL, &err);
		NUI_CHECK_CL_ERR(err);
	}

	m_volume_paramsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(TsdfParams), &m_tsdf_params, &err);
	NUI_CHECK_CL_ERR(err);
	m_MB_numVertsTableCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, 256 * sizeof(int), sNumVertsTable, &err);
	NUI_CHECK_CL_ERR(err);
	m_MB_triTableCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, 256 * 16 * sizeof(int), sTriTable, &err);
	NUI_CHECK_CL_ERR(err);
}


void NuiKinfuOpenCLScene::ReleaseBuffer()
{
	if (m_vertexSumCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_vertexSumCL);
		NUI_CHECK_CL_ERR(err);
		m_vertexSumCL = NULL;
	}
	if (m_volumeCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_volumeCL);
		NUI_CHECK_CL_ERR(err);
		m_volumeCL = NULL;
	}
	if (m_colorVolumeCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_colorVolumeCL);
		NUI_CHECK_CL_ERR(err);
		m_colorVolumeCL = NULL;
	}
	if (m_volume_paramsCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_volume_paramsCL);
		NUI_CHECK_CL_ERR(err);
		m_volume_paramsCL = NULL;
	}
	if (m_MB_numVertsTableCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_MB_numVertsTableCL);
		NUI_CHECK_CL_ERR(err);
		m_MB_numVertsTableCL = NULL;
	}
	if (m_MB_triTableCL) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_MB_triTableCL);
		NUI_CHECK_CL_ERR(err);
		m_MB_triTableCL = NULL;
	}
}

/** \brief Function that integrates volume if volume element contains: 2 bytes for round(tsdf*SHORT_MAX) and 2 bytes for integer weight.*/
bool    NuiKinfuOpenCLScene::integrateVolume(
	NuiKinfuFrame*			pFrame,
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

	cl_mem floatDepthsCL = pCLFrame->GetDepthBuffer();
	cl_mem cameraParamsCL = pCLCamera->GetCameraParamsBuffer();
	if(!floatDepthsCL || !cameraParamsCL || !transformCL)
		return false;

	//cl_mem normalsCL = pCLFeedbackFrame->GetNormalBuffer();
	cl_mem colorsCL = pCLFrame->GetColorBuffer();
	UINT nWidth = pCLFrame->GetWidth();
	UINT nHeight = pCLFrame->GetHeight();

	// Get the kernel
	cl_kernel scaleDepthsKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_SCALE_DEPTHS);
	assert(scaleDepthsKernel);
	if (!scaleDepthsKernel)
	{
		NUI_ERROR("Get kernel 'E_SCALE_DEPTHS' failed!\n");
		return false;
	}

	cl_kernel integrateKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_INTEGRATE_TSDF_VOLUME);
	assert(integrateKernel);
	if (!integrateKernel)
	{
		NUI_ERROR("Get kernel 'E_INTEGRATE_TSDF_VOLUME' failed!\n");
		return false;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(scaleDepthsKernel, idx++, sizeof(cl_mem), &floatDepthsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(scaleDepthsKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	size_t kernelGlobalSize[2] = { nWidth, nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		scaleDepthsKernel,
		2,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	Vector3i voxelWrap = getVoxelWrap();

	// Set kernel arguments
	idx = 0;
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &floatDepthsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &colorsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), NULL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &transformCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_int3), voxelWrap.data());
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &m_volumeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &m_volume_paramsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_mem), &m_colorVolumeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(integrateKernel, idx++, sizeof(cl_uchar), &m_config.max_color_weight);
	NUI_CHECK_CL_ERR(err);

#ifdef _GPU_PROFILER
	cl_event timing_event;
	cl_ulong time_start, time_end;
#endif

	// Run kernel to calculate 
	kernelGlobalSize[0] = (size_t)m_config.resolution[0];
	kernelGlobalSize[1] = (size_t)m_config.resolution[1];
	err = clEnqueueNDRangeKernel(
		queue,
		integrateKernel,
		2,
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

#ifdef _DEBUG
	err = clFinish(queue);
	NUI_CHECK_CL_ERR(err);
#endif
#ifdef _GPU_PROFILER
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_START, sizeof(time_start), &time_start, NULL);
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_END, sizeof(time_end), &time_end, NULL);
	std::cout << "integration:" << (time_end - time_start) << std::endl;
	clReleaseEvent(timing_event);
#endif
	setDirty();

	return true;
}


void    NuiKinfuOpenCLScene::raycastRender(
	NuiKinfuFeedbackFrame*	pFeedbackFrame,
	NuiKinfuCameraState*	pCameraState)
{
	if(!pFeedbackFrame)
		return;
	NuiKinfuOpenCLFeedbackFrame* pCLFeedbackFrame = dynamic_cast<NuiKinfuOpenCLFeedbackFrame*>(pFeedbackFrame);
	if(!pCLFeedbackFrame)
		return;

	cl_mem renderVerticesCL = pCLFeedbackFrame->GetVertexBuffer();
	cl_mem renderNormalsCL = pCLFeedbackFrame->GetNormalBuffer();
	cl_mem renderColorsCL = pCLFeedbackFrame->GetColorBuffer();
	if(!renderVerticesCL || !renderNormalsCL)
		return;

	if(!pCameraState)
		return;
	NuiKinfuOpenCLCameraState* pCLCamera = dynamic_cast<NuiKinfuOpenCLCameraState*>(pCameraState);
	if(!pCLCamera)
		return ;

	cl_mem cameraParamsCL = pCLCamera->GetCameraParamsBuffer();
	cl_mem transformCL = pCLCamera->GetCameraTransformBuffer();
	if(!cameraParamsCL || !transformCL)
		return;

	// Get the kernel
	cl_kernel raycastKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_RAY_CAST);
	assert(raycastKernel);
	if (!raycastKernel)
	{
		NUI_ERROR("Get kernel 'E_RAY_CAST' failed!\n");
		return;
	}

	Vector3i voxelWrap = getVoxelWrap();

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &m_volumeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &m_colorVolumeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_mem), &m_volume_paramsCL);
	NUI_CHECK_CL_ERR(err);
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
	err = clSetKernelArg(raycastKernel, idx++, sizeof(cl_int3), voxelWrap.data());
	NUI_CHECK_CL_ERR(err);


#ifdef _GPU_PROFILER
	cl_event timing_event;
	cl_ulong time_start, time_end;
#endif
	// Run kernel to calculate 
	size_t kernelGlobalSize[2] = { pFeedbackFrame->GetWidth(), pFeedbackFrame->GetHeight() };
	err = clEnqueueNDRangeKernel(
		queue,
		raycastKernel,
		2,
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

#ifdef _GPU_PROFILER
	clFinish(queue);
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_START, sizeof(time_start), &time_start, NULL);
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_END, sizeof(time_end), &time_end, NULL);
	std::cout << "raycast:" << (time_end - time_start) << std::endl;
	clReleaseEvent(timing_event);
#endif
}

bool NuiKinfuOpenCLScene::Volume2CLVertices(NuiCLMappableData* pCLData)
{
	assert(pCLData);
	if(!pCLData)
		return false;

	if(!m_volumeCL || !m_vertexSumCL)
		return false;

	if(!m_dirty)
		return false;
	clearDirty();

	cl_kernel fetchKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_FETCH_VOLUME);
	assert(fetchKernel);
	if (!fetchKernel)
	{
		NUI_ERROR("Get kernel 'E_FETCH_VOLUME' failed!\n");
		return false;
	}

	NuiTimeLog::instance().tick(sVolume2Vertices);

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
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &m_volumeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &m_colorVolumeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &m_volume_paramsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_int), &m_config.resolution(2));
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
	size_t kernelGlobalSize[2] = { (size_t)m_config.resolution(0), (size_t)m_config.resolution(1) };
	size_t local_ws[2] = {1, 1};
	err = clEnqueueNDRangeKernel(
		queue,
		fetchKernel,
		2,
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

#ifdef _GPU_PROFILER
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_START, sizeof(time_start), &time_start, NULL);
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_END, sizeof(time_end), &time_end, NULL);
	std::cout << "volume traversal:" << (time_end - time_start) << std::endl;
	clReleaseEvent(timing_event);
#endif

	//Push the cached point cloud
	/*m_cachedPointCloud.readLock();
	const int cachedVertexSize = m_cachedPointCloud.pointSize();
	const int point_count = cachedVertexSize + vertex_sum;
	if(positions.size() != point_count)
		positions.resize(point_count);
	memcpy((void*)(positions.data()+vertex_sum), m_cachedPointCloud.getVertices(), cachedVertexSize*sizeof(SgVec3f));
	if(colors.size() != point_count)
		colors.resize(point_count);
	memcpy((void*)(colors.data()+vertex_sum), m_cachedPointCloud.getColors(), cachedVertexSize*sizeof(SgVec4f));
	m_cachedPointCloud.readUnlock();*/

	NuiMappableAccessor::asVectorImpl(pCLData->TriangleIndices())->data().clear();
	NuiMappableAccessor::asVectorImpl(pCLData->WireframeIndices())->data().clear();
	if(pCLData->PointIndices().size() != MAX_OUTPUT_VERTEX_SIZE)
	{
		std::vector<unsigned int>& clPointIndices =
			NuiMappableAccessor::asVectorImpl(pCLData->PointIndices())->data();
		clPointIndices.resize(MAX_OUTPUT_VERTEX_SIZE);
		for (int i = 0; i < MAX_OUTPUT_VERTEX_SIZE; ++i)
		{
			clPointIndices[i] = i;
		}
		pCLData->SetIndexingDirty(true);
	}

	// Set bounding box
	const Vector3f& voxelSizeMeters = getVoxelSize();
	pCLData->SetBoundingBox(SgVec3f(
		-m_config.dimensions[0]/2,
		-m_config.dimensions[1]/2,
		-m_config.dimensions[2]/2),
		SgVec3f(
		m_config.dimensions[0]/2,
		m_config.dimensions[1]/2,
		m_config.dimensions[2]/2));

	pCLData->SetStreamDirty(true);

	NuiTimeLog::instance().tock(sVolume2Vertices);
	return true;
}

bool NuiKinfuOpenCLScene::Volume2CLMesh(NuiCLMappableData* pCLData)
{
	assert(pCLData);
	if(!pCLData)
		return false;

	if(!m_volumeCL || !m_vertexSumCL)
		return false;

	if(!m_dirty)
		return true;
	clearDirty();
	
	cl_kernel marchingCubeKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_MARCHING_CUBE);
	assert(marchingCubeKernel);
	if (!marchingCubeKernel)
	{
		NUI_ERROR("Get kernel 'E_MARCHING_CUBE' failed!\n");
		return false;
	}

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
	err = clSetKernelArg(marchingCubeKernel, idx++, sizeof(cl_mem), &m_volumeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(marchingCubeKernel, idx++, sizeof(cl_mem), &m_volume_paramsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(marchingCubeKernel, idx++, sizeof(cl_mem), &m_colorVolumeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(marchingCubeKernel, idx++, sizeof(cl_mem), &positionsGL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(marchingCubeKernel, idx++, sizeof(cl_mem), &colorsGL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(marchingCubeKernel, idx++, sizeof(cl_mem), &m_MB_numVertsTableCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(marchingCubeKernel, idx++, sizeof(cl_mem), &m_MB_triTableCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(marchingCubeKernel, idx++, sizeof(cl_mem), &m_vertexSumCL);
	NUI_CHECK_CL_ERR(err);

#ifdef _GPU_PROFILER
	cl_event timing_event;
	cl_ulong time_start, time_end;
#endif
	// Run kernel to calculate 
	size_t kernelGlobalSize[2] = { (size_t)(m_config.resolution(0)-1), (size_t)(m_config.resolution(1)-1) };
	size_t local_ws[2] = {1, 1};
	err = clEnqueueNDRangeKernel(
		queue,
		marchingCubeKernel,
		2,
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

#ifdef _GPU_PROFILER
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_START, sizeof(time_start), &time_start, NULL);
	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_END, sizeof(time_end), &time_end, NULL);
	std::cout << "marching cube:" << (time_end - time_start) << std::endl;
	clReleaseEvent(timing_event);
#endif

	if(vertex_sum <= 0 || vertex_sum > MAX_OUTPUT_VERTEX_SIZE)
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
	pCLData->SetBoundingBox(SgVec3f(-m_config.dimensions[0]/2, -m_config.dimensions[1]/2, -m_config.dimensions[2]/2), SgVec3f(m_config.dimensions[0]/2, m_config.dimensions[1]/2, m_config.dimensions[2]/2));
	
	pCLData->SetStreamDirty(true);

	return true;
}

SgVec3f vertex_interp(const SgVec3f& p0, const SgVec3f& p1, short tsdf0, short tsdf1)
{
	float f0 = (float)tsdf0 / 32766.0f;
	float f1 = (float)tsdf1 / 32766.0f;
	float t = (0.f - f0) / (f1 - f0 + 1e-15f);
	t = std::max(std::min(t, 1.0f), 0.0f);
	return (p0 + (p1 - p0) * t);
}

bool	NuiKinfuOpenCLScene::Volume2Mesh(NuiMeshShape* pMesh)
{
	assert(pMesh);
	if(!pMesh)
		return false;

	if(!m_volumeCL)
		return false;

	cl_short2* pVolumeData = new cl_short2[m_config.resolution[0]*m_config.resolution[1]*m_config.resolution[2]];

	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	err = clEnqueueReadBuffer(
		queue,
		m_volumeCL,
		CL_TRUE,//blocking
		0,
		m_config.resolution[0]*m_config.resolution[1]*m_config.resolution[2] * sizeof(cl_short2),
		pVolumeData,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	std::map<INT64, int> indexMap;
	int nVertexIndex = 0;
	for (int y = 0; y < m_config.resolution[1]-1; ++y)
	{
		for (int x = 0; x < m_config.resolution[0]-1; ++x)
		{
			const int voxel_id_base0 = ((y * m_config.resolution[0]) + x) * m_config.resolution[2];
			const int voxel_id_base1 = voxel_id_base0 + m_config.resolution[2];
			const int voxel_id_base3 = voxel_id_base0 + m_config.resolution[0] * m_config.resolution[2];
			const int voxel_id_base2 = voxel_id_base3 + m_config.resolution[2];

			for (int z = 0; z < m_config.resolution[2]-1; ++z)
			{
				int voxel_id[8];
				voxel_id[0] = voxel_id_base0 + z;
				voxel_id[1] = voxel_id_base1 + z;
				voxel_id[2] = voxel_id_base2 + z;
				voxel_id[3] = voxel_id_base3 + z;
				voxel_id[4] = voxel_id_base0 + z + 1;
				voxel_id[5] = voxel_id_base1 + z + 1;
				voxel_id[6] = voxel_id_base2 + z + 1;
				voxel_id[7] = voxel_id_base3 + z + 1;

				cl_short2 tsdf0 = pVolumeData[voxel_id[0]];
				if(tsdf0.y <= 0)
					continue;
				cl_short2 tsdf1 = pVolumeData[voxel_id[1]];
				if(tsdf1.y <= 0)
					continue;
				cl_short2 tsdf2 = pVolumeData[voxel_id[2]];
				if(tsdf2.y <= 0)
					continue;
				cl_short2 tsdf3 = pVolumeData[voxel_id[3]];
				if(tsdf3.y <= 0)
					continue;
				cl_short2 tsdf4 = pVolumeData[voxel_id[4]];
				if(tsdf4.y <= 0)
					continue;
				cl_short2 tsdf5 = pVolumeData[voxel_id[5]];
				if(tsdf5.y <= 0)
					continue;
				cl_short2 tsdf6 = pVolumeData[voxel_id[6]];
				if(tsdf6.y <= 0)
					continue;
				cl_short2 tsdf7 = pVolumeData[voxel_id[7]];
				if(tsdf7.y <= 0)
					continue;

				// calculate flag indicating if each vertex is inside or outside isosurface
				int cubeindex;
				cubeindex = (int)(tsdf0.x < 0);
				cubeindex += (int)(tsdf1.x < 0) * 2;
				cubeindex += (int)(tsdf2.x < 0) * 4;
				cubeindex += (int)(tsdf3.x < 0) * 8;
				cubeindex += (int)(tsdf4.x < 0) * 16;
				cubeindex += (int)(tsdf5.x < 0) * 32;
				cubeindex += (int)(tsdf6.x < 0) * 64;
				cubeindex += (int)(tsdf7.x < 0) * 128;

				int numVerts = sNumVertsTable[cubeindex];
				if(0 == numVerts)
					continue;

				SgVec3f v[8];
				v[0] = getNodeCoo(x, y, z);
				v[1] = getNodeCoo(x + 1, y, z);
				v[2] = getNodeCoo(x + 1, y + 1, z);
				v[3] = getNodeCoo(x, y + 1, z);
				v[4] = getNodeCoo(x, y, z + 1);
				v[5] = getNodeCoo(x + 1, y, z + 1);
				v[6] = getNodeCoo(x + 1, y + 1, z + 1);
				v[7] = getNodeCoo(x, y + 1, z + 1);

				SgVec3f vertlist[12];
				vertlist[0] = vertex_interp (v[0], v[1], tsdf0.x, tsdf1.x);
				vertlist[1] = vertex_interp (v[1], v[2], tsdf1.x, tsdf2.x);
				vertlist[2] = vertex_interp (v[2], v[3], tsdf2.x, tsdf3.x);
				vertlist[3] = vertex_interp (v[3], v[0], tsdf3.x, tsdf0.x);
				vertlist[4] = vertex_interp (v[4], v[5], tsdf4.x, tsdf5.x);
				vertlist[5] = vertex_interp (v[5], v[6], tsdf5.x, tsdf6.x);
				vertlist[6] = vertex_interp (v[6], v[7], tsdf6.x, tsdf7.x);
				vertlist[7] = vertex_interp (v[7], v[4], tsdf7.x, tsdf4.x);
				vertlist[8] = vertex_interp (v[0], v[4], tsdf0.x, tsdf4.x);
				vertlist[9] = vertex_interp (v[1], v[5], tsdf1.x, tsdf5.x);
				vertlist[10] = vertex_interp (v[2], v[6], tsdf2.x, tsdf6.x);
				vertlist[11] = vertex_interp (v[3], v[7], tsdf3.x, tsdf7.x);

				for (int i = 0; i < numVerts; i += 3)
				{
					int v1 = sTriTable[cubeindex][i];
					int v2 = sTriTable[cubeindex][i+1];
					int v3 = sTriTable[cubeindex][i+2];

					if(v1 < 0 || v2 < 0 || v3 < 0)
						continue;

					int wire_id0 = sWireIndices[v1][0];
					int wire_id1 = sWireIndices[v1][1];
					INT64 v_id0 = voxel_id[wire_id0];
					INT64 v_id1 = voxel_id[wire_id1];
					INT64 v_compound_id = (v_id0 << 32) + v_id1;
					// The first vertex
					std::map<INT64, int>::iterator itr = indexMap.find(v_compound_id);
					if(itr == indexMap.end())
					{
						pMesh->appendPoint(vertlist[v1]);
						pMesh->appendTriangleIndex(nVertexIndex);

						indexMap.insert(std::pair<INT64, int>(v_compound_id, nVertexIndex));
						nVertexIndex ++ ;
					}
					else
					{
						pMesh->appendTriangleIndex(itr->second);
					}
					// The second vertex
					wire_id0 = sWireIndices[v2][0];
					wire_id1 = sWireIndices[v2][1];
					v_id0 = voxel_id[wire_id0];
					v_id1 = voxel_id[wire_id1];
					v_compound_id = (v_id0 << 32) + v_id1;
					itr = indexMap.find(v_compound_id);
					if(itr == indexMap.end())
					{
						pMesh->appendPoint(vertlist[v2]);
						pMesh->appendTriangleIndex(nVertexIndex);

						indexMap.insert(std::pair<INT64, int>(v_compound_id, nVertexIndex));
						nVertexIndex ++ ;
					}
					else
					{
						pMesh->appendTriangleIndex(itr->second);
					}
					// The third vertex
					wire_id0 = sWireIndices[v3][0];
					wire_id1 = sWireIndices[v3][1];
					v_id0 = voxel_id[wire_id0];
					v_id1 = voxel_id[wire_id1];
					v_compound_id = (v_id0 << 32) + v_id1;
					itr = indexMap.find(v_compound_id);
					if(itr == indexMap.end())
					{
						pMesh->appendPoint(vertlist[v3]);
						pMesh->appendTriangleIndex(nVertexIndex);

						indexMap.insert(std::pair<INT64, int>(v_compound_id, nVertexIndex));
						nVertexIndex ++ ;
					}
					else
					{
						pMesh->appendTriangleIndex(itr->second);
					}
				}
			}
		}
	}

	delete[] pVolumeData;
	return true;
}

//bool NuiKinfuVolume::VolumeBuffer2(NuiCLMappableData* pCLData)
//{
//	assert(pCLData);
//	if(!pCLData)
//		return false;
//
//	if(!m_integrationDirty)
//		return true;
//
//	m_integrationDirty = false;
//
//	cl_kernel volumeTraversalKernel =
//		NuiOpenCLKernelManager::instance().acquireKernel(E_VOLUME_TRAVERSAL);
//	assert(volumeTraversalKernel);
//	if (!volumeTraversalKernel)
//	{
//		NUI_ERROR("Get kernel 'E_VOLUME_TRAVERSAL' failed!\n");
//		return false;
//	}
//
//	cl_kernel volume2VertexKernel =
//		NuiOpenCLKernelManager::instance().acquireKernel(E_VOLUME2VERTEX);
//	assert(volume2VertexKernel);
//	if (!volume2VertexKernel)
//	{
//		NUI_ERROR("Get kernel 'E_VOLUME2VERTEX' failed!\n");
//		return false;
//	}
//
//	const Vector3i& volume_resolution = m_tsdf_volume->getResolution();
//	const UINT voxel_num_size = volume_resolution(0) * volume_resolution(1);
//	cl_short* voxel_num_buffer = new cl_short[voxel_num_size];
//	cl_mem volume_data = m_tsdf_volume->data();
//
//	cl_int           err = CL_SUCCESS;
//	cl_context       context = NuiOpenCLGlobal::instance().clContext();
//	cl_mem voxelNumsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_WRITE_ONLY, voxel_num_size*sizeof(cl_short), NULL, &err);
//	NUI_CHECK_CL_ERR(err);
//
//	// OpenCL command queue and device
//	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();
//
//	// Set kernel arguments
//	cl_uint idx = 0;
//	err = clSetKernelArg(volumeTraversalKernel, idx++, sizeof(cl_mem), &volume_data);
//	NUI_CHECK_CL_ERR(err);
//	err = clSetKernelArg(volumeTraversalKernel, idx++, sizeof(cl_uint), &volume_resolution(2));
//	NUI_CHECK_CL_ERR(err);
//	err = clSetKernelArg(volumeTraversalKernel, idx++, sizeof(cl_mem), &voxelNumsCL);
//	NUI_CHECK_CL_ERR(err);
//
//#ifdef _GPU_PROFILER
//	cl_event timing_event;
//	cl_ulong time_start, time_end;
//#endif
//	// Run kernel to calculate 
//	size_t kernelGlobalSize[2] = { (size_t)volume_resolution(0), (size_t)volume_resolution(1) };
//	err = clEnqueueNDRangeKernel(
//		queue,
//		volumeTraversalKernel,
//		2,
//		nullptr,
//		kernelGlobalSize,
//		nullptr,
//		0,
//		NULL,
//#ifdef _GPU_PROFILER
//		&timing_event
//#else
//		NULL
//#endif
//		);
//	NUI_CHECK_CL_ERR(err);
//
//	err = clEnqueueReadBuffer(
//		queue,
//		voxelNumsCL,
//		CL_TRUE,//blocking
//		0,
//		voxel_num_size*sizeof(cl_short),
//		voxel_num_buffer,
//		0,
//		NULL,
//		NULL
//		);
//	NUI_CHECK_CL_ERR(err);
//
//	cl_uint* voxel_sum_buffer = new cl_uint[voxel_num_size];
//	cl_uint voxel_sum = 0;
//	int voxel_offset_x = (int)volume_resolution(0);
//	int voxel_offset_y = (int)volume_resolution(1);
//	int voxel_end_x = -1;
//	int voxel_end_y = -1;
//	for (int y = 0; y < (int)volume_resolution(1); y ++)
//	{
//		for (int x = 0; x < (int)volume_resolution(0); x ++)
//		{
//			int id = y * volume_resolution(0) +x;
//			if(voxel_num_buffer[id] > 0)
//			{
//				if(voxel_offset_y > y)
//					voxel_offset_y = y;
//				if(voxel_offset_x > x)
//					voxel_offset_x = x;
//				if(voxel_end_x < x)
//					voxel_end_x = x;
//				if(voxel_end_y < y)
//					voxel_end_y = y;
//
//				voxel_sum += voxel_num_buffer[id];
//			}
//			voxel_sum_buffer[id] = voxel_sum;
//		}
//	}
//
//	SafeDeleteArray(voxel_num_buffer);
//	err = NuiGPUMemManager::instance().ReleaseMemObjectCL(voxelNumsCL);
//	NUI_CHECK_CL_ERR(err);
//
//	std::vector<unsigned int>& clTriangleIndices =
//		NuiMappableAccessor::asVectorImpl(pCLData->TriangleIndices())->data();
//	clTriangleIndices.clear();
//
//	std::vector<unsigned int>& clWireframeIndices =
//		NuiMappableAccessor::asVectorImpl(pCLData->WireframeIndices())->data();
//	clWireframeIndices.clear();
//
//	std::vector<unsigned int>& clPointIndices =
//		NuiMappableAccessor::asVectorImpl(pCLData->PointIndices())->data();
//	if(clPointIndices.size() != voxel_sum)
//	{
//		clPointIndices.resize(voxel_sum);
//		for (UINT i = 0; i < voxel_sum; ++i)
//		{
//			clPointIndices[i] = i;
//		}
//		pCLData->SetIndexingDirty(true);
//	}
//
//	pCLData->SetStreamDirty(true);
//
//	std::vector<SgVec3f>& positions = NuiMappableAccessor::asVectorImpl(pCLData->PositionStream())->data();
//	if(voxel_sum == 0)
//	{
//		positions.clear();
//		SafeDeleteArray(voxel_sum_buffer);
//		return true;
//	}
//	if(positions.size() != voxel_sum)
//		positions.resize(voxel_sum);
//
//	int voxel_sum_size = (voxel_end_y-voxel_offset_y+1) * (voxel_end_x-voxel_offset_x+1);
//	int first_voxel = voxel_offset_y * volume_resolution(1) + voxel_offset_x;
//	cl_mem voxelSumsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, voxel_sum_size*sizeof(cl_uint), voxel_sum_buffer+first_voxel, &err);
//	NUI_CHECK_CL_ERR(err);
//
//	cl_mem voxelsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_WRITE_ONLY, voxel_sum*3*sizeof(float), NULL, &err);
//	NUI_CHECK_CL_ERR(err);
//
//	idx = 0;
//	err = clSetKernelArg(volume2VertexKernel, idx++, sizeof(cl_mem), &volume_data);
//	NUI_CHECK_CL_ERR(err);
//	err = clSetKernelArg(volume2VertexKernel, idx++, sizeof(cl_mem), &m_volume_paramsCL);
//	NUI_CHECK_CL_ERR(err);
//	err = clSetKernelArg(volume2VertexKernel, idx++, sizeof(cl_mem), &voxelSumsCL);
//	NUI_CHECK_CL_ERR(err);
//	err = clSetKernelArg(volume2VertexKernel, idx++, sizeof(cl_uint), &voxel_offset_x);
//	NUI_CHECK_CL_ERR(err);
//	err = clSetKernelArg(volume2VertexKernel, idx++, sizeof(cl_uint), &voxel_offset_y);
//	NUI_CHECK_CL_ERR(err);
//	err = clSetKernelArg(volume2VertexKernel, idx++, sizeof(cl_mem), &voxelsCL);
//	NUI_CHECK_CL_ERR(err);
//
//	// Run kernel to calculate 
//	kernelGlobalSize[0] = voxel_end_x - voxel_offset_x + 1;
//	kernelGlobalSize[1] = voxel_end_y - voxel_offset_y + 1;
//	err = clEnqueueNDRangeKernel(
//		queue,
//		volume2VertexKernel,
//		2,
//		nullptr,
//		kernelGlobalSize,
//		nullptr,
//		0,
//		NULL,
//		NULL
//		);
//	NUI_CHECK_CL_ERR(err);
//
//	clEnqueueReadBuffer(
//		queue,
//		voxelsCL,
//		CL_TRUE,//blocking
//		0,
//		voxel_sum * 3 * sizeof(float),
//		positions.data(),
//		0,
//		NULL,
//		NULL
//		);
//	NUI_CHECK_CL_ERR(err);
//
//	SafeDeleteArray(voxel_sum_buffer);
//	err = NuiGPUMemManager::instance().ReleaseMemObjectCL(voxelSumsCL);
//	NUI_CHECK_CL_ERR(err);
//	err = NuiGPUMemManager::instance().ReleaseMemObjectCL(voxelsCL);
//	NUI_CHECK_CL_ERR(err);
//
//#ifdef _GPU_PROFILER
//	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_START, sizeof(time_start), &time_start, NULL);
//	clGetEventProfilingInfo(timing_event, CL_PROFILING_COMMAND_END, sizeof(time_end), &time_end, NULL);
//	std::cout << "volume traversal:" << (time_end - time_start) << std::endl;
//	clReleaseEvent(timing_event);
//#endif
//#ifdef _DEBUG
//	//if(m_globalTime == 1 && level_index == 2)
//	/*{
//		for (int y = 0; y < volume_resolution(1); y ++)
//		{
//			for (int x = 0; x < volume_resolution(0); x ++)
//			{
//				int index = x + y * volume_resolution(0);
//				SgVec3f value = positions.at(index);
//				CvScalar color = CvScalar(value[0], value[1], value[2]);
//				cvSet2D(m_pDebugImg, y, x, color);
//			}
//		}
//		cvShowImage( "DebugImage", m_pDebugImg );
//		char cvKey = cvWaitKey( 20 );
//	}*/
//#endif
//
//	return true;
//}