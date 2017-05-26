#include "NuiSLAMController.h"

#include "SLAM/VisualOdometry/DeviceSpecific/OpenCL/NuiKinfuOpenCLShiftScene.h"
#include "SLAM/VisualOdometry/DeviceSpecific/OpenCL/NuiKinfuOpenCLHashScene.h"

#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLBufferFactory.h"
#include "Shape/NuiCLMappableData.h"
#include "Foundation/NuiLogger.h"

using namespace NuiSLAMEngine;

NuiSLAMController::NuiSLAMController()
	: m_pScene(NULL)
{
}

NuiSLAMController::~NuiSLAMController()
{
	SafeDelete(m_pScene);
}

void	NuiSLAMController::resetScene()
{
	if (m_pScene)
		m_pScene->reset();
	m_cachedPointCloud.clear();
}

void	NuiSLAMController::setVolume(float voxelSize, int sceneMode)
{
	bool bIsTracking = m_tracker.isThreadOn();
	if(bIsTracking)
		m_tracker.stopThread();
	SafeDelete(m_pScene);

	NuiKinfuVolumeConfig volumeConfig;
	switch (sceneMode)
	{
	case eScene_HashingVolume:
	{
		NuiHashingSDFConfig sdfConfig;
		sdfConfig.m_bUseSwapping = false;
		sdfConfig.m_bUseForwardRender = true;
		sdfConfig.m_virtualVoxelSize = voxelSize;
		//sdfConfig.m_truncation = 5.0f * sdfConfig.m_virtualVoxelSize;
		//sdfConfig.m_truncScale = 2.5f * sdfConfig.m_virtualVoxelSize;
		//NuiHashingRaycastConfig raycastConfig;
		m_pScene = new NuiKinfuOpenCLHashScene(sdfConfig);
		break;
	}
	case eScene_ShiftingVolume:
		volumeConfig.bIsDynamic = true;
	case eScene_FusionVolume:
	default:
		volumeConfig.dimensions = Vector3f::Constant(3.0f);
		volumeConfig.resolution = Vector3i::Constant(int(3.0f / voxelSize));
		volumeConfig.translateBasis = m_tracker.getTranslateBasis();
		m_pScene = volumeConfig.bIsDynamic ? new NuiKinfuOpenCLShiftScene(volumeConfig, &m_cachedPointCloud) : new NuiKinfuOpenCLScene(volumeConfig);
		break;
	}

	m_tracker.setScene(m_pScene);
	if (bIsTracking)
		m_tracker.startThread();
}

void	NuiSLAMController::log(const std::string& fileName) const
{
	m_tracker.log(fileName);
	if(m_pScene)
		m_pScene->log(fileName);
}

bool	NuiSLAMController::evaluateCLData(NuiCLMappableData* pCLData, int drawMode)
{
	if(!pCLData)
		return false;

	m_tracker.evaluateCLData(pCLData);

	bool returnStatus = false;
	if( m_pScene )
	{
		boost::mutex::scoped_lock volumeLock(m_tracker.getMutex());
		switch (drawMode)
		{
		case eDraw_PointCloud:
			returnStatus = m_pScene->Volume2CLVertices(pCLData);
			break;
		case eDraw_PolygonMesh:
		{
			returnStatus = m_pScene->Volume2CLVertices(pCLData);
			if (returnStatus)
			{
				CachePointCloud(pCLData);
			}
			break;
		}
		case eDraw_RealtimeMesh:
			returnStatus = m_pScene->Volume2CLMesh(pCLData);
			break;
		case eDraw_None:
		default:
			break;
		}
		volumeLock.unlock();
	}

	return returnStatus;
}

bool	NuiSLAMController::getMesh(NuiMeshShape* pMesh)
{
	if(!m_pScene ||  !pMesh)
		return false;

	return m_pScene->Volume2Mesh(pMesh);
}

void	NuiSLAMController::CachePointCloud(NuiCLMappableData* pCLData)
{
	assert(pCLData);
	if (!pCLData)
		return;

	int vertex_sum = pCLData->PointIndices().size();
	if (vertex_sum <= 0)
		return;

	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	cl_mem positionsGL = NuiOpenCLBufferFactory::asPosition3fBufferCL(pCLData->PositionStream());
	cl_mem colorsGL = NuiOpenCLBufferFactory::asColor4fBufferCL(pCLData->ColorStream());
	// Acquire OpenGL objects before use
	cl_mem glObjs[] = {
		positionsGL,
		colorsGL
	};

	openclutil::enqueueAcquireHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);

	m_cachedPointCloud.writeLock();

	int originalSize = m_cachedPointCloud.pointSize();
	m_cachedPointCloud.resizePoints(originalSize + vertex_sum);
	err = clEnqueueReadBuffer(
		queue,
		positionsGL,
		CL_FALSE,//blocking
		0,
		vertex_sum * 3 * sizeof(float),
		(void*)(m_cachedPointCloud.getVertices() + originalSize),
		0,
		NULL,
		NULL
	);
	NUI_CHECK_CL_ERR(err);

	err = clEnqueueReadBuffer(
		queue,
		colorsGL,
		CL_TRUE,//blocking
		0,
		vertex_sum * 4 * sizeof(float),
		(void*)(m_cachedPointCloud.getColors() + originalSize),
		0,
		NULL,
		NULL
	);
	NUI_CHECK_CL_ERR(err);

	m_cachedPointCloud.writeUnlock();

	// Release OpenGL objects
	openclutil::enqueueReleaseHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);

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

}
