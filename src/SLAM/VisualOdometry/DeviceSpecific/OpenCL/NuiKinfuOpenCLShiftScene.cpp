#include "NuiKinfuOpenCLShiftScene.h"

#include "NuiKinfuOpenCLFrame.h"
#include "NuiKinfuOpenCLFeedbackFrame.h"
#include "NuiKinfuOpenCLCameraState.h"

#include "../../NuiKinfuVertexCache.h"
#include "../NuiKinfuCameraState.h"
#include "Foundation/NuiDebugMacro.h"
#include "Foundation/NuiTimeLog.h"
#include "Foundation/NuiLogger.h"
#include "NuiMarchingCubeTable.h"
#include "Shape/NuiCLMappableData.h"
#include "Shape\NuiMeshShape.h"

#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"
#include "OpenCLUtilities/NuiOpenCLBufferFactory.h"

static const std::string sVolume2Vertices("Volume2Vertices");

NuiKinfuOpenCLShiftScene::NuiKinfuOpenCLShiftScene(const NuiKinfuVolumeConfig& config, NuiKinfuVertexCache* pCache)
	: NuiKinfuOpenCLScene(config)
	, m_pCachedPointCloud(pCache)
{
	m_voxel_offset = Vector3i::Zero();
}

NuiKinfuOpenCLShiftScene::~NuiKinfuOpenCLShiftScene()
{
	ReleaseBuffer();
}

/*virtual*/
void NuiKinfuOpenCLShiftScene::reset()
{
	m_voxel_offset = Vector3i::Zero();

	NuiKinfuOpenCLScene::reset();
}

/*virtual*/
Vector3i NuiKinfuOpenCLShiftScene::getVoxelWrap() const
{
	Vector3i voxelWrap = m_voxel_offset;

	if(voxelWrap(0) < 0)
		voxelWrap(0) = m_config.resolution(0) - ((-voxelWrap(0)) % m_config.resolution(0));
	if(voxelWrap(1) < 0)
		voxelWrap(1) = m_config.resolution(1) - ((-voxelWrap(1)) % m_config.resolution(1));
	if(voxelWrap(2) < 0)
		voxelWrap(2) = m_config.resolution(2) - ((-voxelWrap(2)) % m_config.resolution(2));

	return voxelWrap;
}

Vector3f NuiKinfuOpenCLShiftScene::getVoxelOffsetSize() const
{
	const Vector3f& voxelSizeMeters = getVoxelSize();
	return Vector3f(
		(float)m_voxel_offset[0] * voxelSizeMeters[0],
		(float)m_voxel_offset[1] * voxelSizeMeters[1],
		(float)m_voxel_offset[2] * voxelSizeMeters[2]
		);
}

float	NuiKinfuOpenCLShiftScene::getVoxelLeafSize() const
{
	return std::max(m_tsdf_params.cell_size[0],
		std::max(m_tsdf_params.cell_size[1],
			m_tsdf_params.cell_size[2]));
}

void NuiKinfuOpenCLShiftScene::clearSlice(const Vector3i&	voxelWrap, const Vector3i&	voxelRange)
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

	cl_uint idx = 0;
	err = clSetKernelArg(initializeKernel, idx++, sizeof(cl_mem), &m_volumeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(initializeKernel, idx++, sizeof(cl_mem), &m_colorVolumeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(initializeKernel, idx++, sizeof(cl_mem), &m_volume_paramsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(initializeKernel, idx++, sizeof(cl_int3), voxelWrap.data());
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	size_t kernelGlobalSize[3] = { voxelRange(0), voxelRange(1), voxelRange(2) };
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
}

void NuiKinfuOpenCLShiftScene::fetchSlice(const Vector3i&	voxelWrap, const Vector3i&	voxelRange)
{
	if (!m_pCachedPointCloud)
		return;

	// Get the kernel
	cl_kernel fetchKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_FETCH_SHIFT_VOLUME);
	assert(fetchKernel);
	if (!fetchKernel)
	{
		NUI_ERROR("Get kernel 'E_FETCH_SHIFT_VOLUME' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();
	cl_context       context = NuiOpenCLGlobal::instance().clContext();

	cl_mem volumeOutputVerticesCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, voxelRange(0)*voxelRange(1)*voxelRange(2)*sizeof(cl_float3), NULL, &err);
	NUI_CHECK_CL_ERR(err);
	cl_mem volumeOutputColorsCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, voxelRange(0)*voxelRange(1)*voxelRange(2)*sizeof(cl_float4), NULL, &err);
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

	cl_uint idx = 0;
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &m_volumeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &m_colorVolumeCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &m_volume_paramsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_int), &voxelRange(2));
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_int3), voxelWrap.data());
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_int3), m_voxel_offset.data());
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &volumeOutputVerticesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &volumeOutputColorsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_mem), &m_vertexSumCL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	size_t kernelGlobalSize[2] = { (size_t)voxelRange(0), (size_t)voxelRange(1) };
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
		NULL
		);
	NUI_CHECK_CL_ERR(err);

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

	if (vertex_sum > 0)
	{
		m_pCachedPointCloud->writeLock();

		int originalSize = m_pCachedPointCloud->pointSize();
		m_pCachedPointCloud->resizePoints(originalSize + vertex_sum);
		err = clEnqueueReadBuffer(
			queue,
			volumeOutputVerticesCL,
			CL_FALSE,//blocking
			0,
			vertex_sum * 3 * sizeof(float),
			(void*)(m_pCachedPointCloud->getVertices() + originalSize),
			0,
			NULL,
			NULL
		);
		NUI_CHECK_CL_ERR(err);

		err = clEnqueueReadBuffer(
			queue,
			volumeOutputColorsCL,
			CL_FALSE,//blocking
			0,
			vertex_sum * 4 * sizeof(float),
			(void*)(m_pCachedPointCloud->getColors() + originalSize),
			0,
			NULL,
			NULL
		);
		NUI_CHECK_CL_ERR(err);

		m_pCachedPointCloud->writeUnlock();
	}

	err = NuiGPUMemManager::instance().ReleaseMemObjectCL(volumeOutputVerticesCL);
	NUI_CHECK_CL_ERR(err);
	err = NuiGPUMemManager::instance().ReleaseMemObjectCL(volumeOutputColorsCL);
	NUI_CHECK_CL_ERR(err);
}

void NuiKinfuOpenCLShiftScene::fetchAndClearX(int xVoxelTrans)
{
	int offsetX = m_voxel_offset(0);
	if(xVoxelTrans < 0)
		offsetX += xVoxelTrans;
	offsetX = (offsetX >= 0) ? (offsetX % m_config.resolution(0)) : (m_config.resolution(0) - (-offsetX) % m_config.resolution(0));
	Vector3i voxelWrap(offsetX, 0, 0);
	Vector3i voxelTranslation(abs(xVoxelTrans), m_config.resolution(1), m_config.resolution(2));
	fetchSlice(voxelWrap, voxelTranslation);
	clearSlice(voxelWrap, voxelTranslation);

	m_voxel_offset(0) += xVoxelTrans;
}

void NuiKinfuOpenCLShiftScene::fetchAndClearY(int yVoxelTrans)
{
	int offsetY = m_voxel_offset(1);
	if(yVoxelTrans < 0)
		offsetY += yVoxelTrans;
	offsetY = (offsetY >= 0) ? (offsetY % m_config.resolution(1)) : (m_config.resolution(1) - (-offsetY) % m_config.resolution(1));
	Vector3i voxelWrap(0, offsetY, 0);
	Vector3i voxelTranslation(m_config.resolution(0), abs(yVoxelTrans), m_config.resolution(2));
	fetchSlice(voxelWrap, voxelTranslation);
	clearSlice(voxelWrap, voxelTranslation);

	m_voxel_offset(1) += yVoxelTrans;
}

void NuiKinfuOpenCLShiftScene::fetchAndClearZ(int zVoxelTrans)
{
	int offsetZ = m_voxel_offset(2);
	if(zVoxelTrans < 0)
		offsetZ += zVoxelTrans;
	offsetZ = (offsetZ >= 0) ? (offsetZ % m_config.resolution(2)) : (m_config.resolution(2) - (-offsetZ) % m_config.resolution(2));
	Vector3i voxelWrap(0, 0, offsetZ);
	Vector3i voxelTranslation(m_config.resolution(0), m_config.resolution(1), abs(zVoxelTrans));
	fetchSlice(voxelWrap, voxelTranslation);
	clearSlice(voxelWrap, voxelTranslation);

	m_voxel_offset(2) += zVoxelTrans;
}

Vector3f NuiKinfuOpenCLShiftScene::shiftVolume(const Vector3f& translation)
{
	const Vector3f& voxelSizeMeters = getVoxelSize();
	Vector3f shiftTranslation = translation - m_config.translateBasis;

	int xVoxelTrans = 0;
	int yVoxelTrans = 0;
	int zVoxelTrans = 0;

	if(shiftTranslation(0) < 0)
	{
		xVoxelTrans = std::max(-m_config.voxel_shift, (int)std::floor(shiftTranslation(0) / voxelSizeMeters(0)));
	}
	else
	{
		xVoxelTrans = std::min(m_config.voxel_shift, (int)std::floor(shiftTranslation(0) / voxelSizeMeters(0)));
	}

	if(shiftTranslation(1) < 0)
	{
		yVoxelTrans = std::max(-m_config.voxel_shift, (int)std::floor(shiftTranslation(1) / voxelSizeMeters(1)));
	}
	else
	{
		yVoxelTrans = std::min(m_config.voxel_shift, (int)std::floor(shiftTranslation(1) / voxelSizeMeters(1)));
	}

	if(shiftTranslation(2) < 0)
	{
		zVoxelTrans = std::max(-m_config.voxel_shift, (int)std::floor(shiftTranslation(2) / voxelSizeMeters(2)));
	}
	else
	{
		zVoxelTrans = std::min(m_config.voxel_shift, (int)std::floor(shiftTranslation(2) / voxelSizeMeters(2)));
	}

	if(abs(xVoxelTrans) >= m_config.voxel_shift)
	{
		fetchAndClearX(xVoxelTrans);
		shiftTranslation(0) -= xVoxelTrans * voxelSizeMeters(0);
#ifdef _DEBUG
		LOG4CPLUS_DEBUG(NuiLogger::instance().consoleLogger(), "Scene shift x-voxel: " << shiftTranslation(0));
#endif // _DEBUG
	}
	if(abs(yVoxelTrans) >= m_config.voxel_shift)
	{
		fetchAndClearY(yVoxelTrans);
		shiftTranslation(1) -= yVoxelTrans * voxelSizeMeters(1);
#ifdef _DEBUG
		LOG4CPLUS_DEBUG(NuiLogger::instance().consoleLogger(), "Scene shift y-voxel: " << shiftTranslation(1));
#endif // _DEBUG
	}
	if(abs(zVoxelTrans) >= m_config.voxel_shift)
	{
		fetchAndClearZ(zVoxelTrans);
		shiftTranslation(2) -= zVoxelTrans * voxelSizeMeters(2);
#ifdef _DEBUG
		LOG4CPLUS_DEBUG(NuiLogger::instance().consoleLogger(), "Scene shift z-voxel: " << shiftTranslation(2));
#endif // _DEBUG
	}

	return shiftTranslation + m_config.translateBasis;
}

/*virtual*/
bool	NuiKinfuOpenCLShiftScene::integrateVolume(
	NuiKinfuFrame*			pFrame,
	NuiKinfuCameraState*	pCameraState
)
{
	Vector3f translation = pCameraState->GetCameraPos().getLocalTranslation();
	translation = shiftVolume(translation);
	pCameraState->UpdateCameraTransform(pCameraState->GetCameraPos().getRotation(), translation, getVoxelOffsetSize());

	return NuiKinfuOpenCLScene::integrateVolume(pFrame, pCameraState);
}

bool NuiKinfuOpenCLShiftScene::Volume2CLVertices(NuiCLMappableData* pCLData)
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
		NuiOpenCLKernelManager::instance().acquireKernel(E_FETCH_SHIFT_VOLUME);
	assert(fetchKernel);
	if (!fetchKernel)
	{
		NUI_ERROR("Get kernel 'E_FETCH_SHIFT_VOLUME' failed!\n");
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

	cl_int3 voxelWrap;
	voxelWrap.x = voxelWrap.y = voxelWrap.z = 0;

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
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_int3), &voxelWrap);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(fetchKernel, idx++, sizeof(cl_int3), m_voxel_offset.data());
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

	pCLData->SetStreamDirty(true);

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
	if(pCLData->PointIndices().size() != vertex_sum)
	{
		std::vector<unsigned int>& clPointIndices =
			NuiMappableAccessor::asVectorImpl(pCLData->PointIndices())->data();
		clPointIndices.resize(vertex_sum);
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < vertex_sum; ++i)
		{
			clPointIndices[i] = i;
		}
		pCLData->SetIndexingDirty(true);
	}

	// Set bounding box
	const Vector3f& voxelSizeMeters = getVoxelSize();
	pCLData->SetBoundingBox(SgVec3f(
		m_voxel_offset(0)*voxelSizeMeters(0)-m_config.dimensions[0]/2,
		m_voxel_offset(1)*voxelSizeMeters(1)-m_config.dimensions[1]/2,
		m_voxel_offset(2)*voxelSizeMeters(2)),
		SgVec3f(
		m_voxel_offset(0)*voxelSizeMeters(0)+m_config.dimensions[0]/2,
		m_voxel_offset(1)*voxelSizeMeters(1)+m_config.dimensions[1]/2,
		m_voxel_offset(2)*voxelSizeMeters(2)+m_config.dimensions[2]));

	NuiTimeLog::instance().tock(sVolume2Vertices);
	return true;
}

bool NuiKinfuOpenCLShiftScene::Volume2CLMesh(NuiCLMappableData* pCLData)
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
	pCLData->SetBoundingBox(SgVec3f(-m_config.dimensions[0]/2, -m_config.dimensions[1]/2, 0), SgVec3f(m_config.dimensions[0]/2, m_config.dimensions[1]/2, m_config.dimensions[2]));
	
	pCLData->SetStreamDirty(true);

	return true;
}

