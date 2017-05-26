#include "NuiKinfuMainEngine.h"

#include "DeviceSpecific/OpenCL/NuiKinfuOpenCLShiftScene.h"
#include "DeviceSpecific/OpenCL/NuiKinfuOpenCLHashScene.h"

#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLBufferFactory.h"
#include "Shape/NuiCLMappableData.h"
#include "Foundation/NuiTimeLog.h"
#include "Foundation/NuiLogger.h"

static const std::string sTrackingName("TrackingEngine");

using namespace NuiKinfuEngine;

NuiKinfuMainEngine::NuiKinfuMainEngine()
	: m_pScene(NULL)
{
}

NuiKinfuMainEngine::~NuiKinfuMainEngine()
{
	SafeDelete(m_pScene);
}


void	NuiKinfuMainEngine::setVolume(float voxelSize, int sceneMode)
{
	SafeDelete(m_pScene);

	NuiKinfuVolumeConfig volumeConfig;
	switch (sceneMode)
	{
	case NuiKinfuEngine::NuiKinfuMainEngine::eScene_HashingVolume:
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
	case NuiKinfuEngine::NuiKinfuMainEngine::eScene_ShiftingVolume:
		volumeConfig.bIsDynamic = true;
	case NuiKinfuEngine::NuiKinfuMainEngine::eScene_FusionVolume:
	default:
		volumeConfig.dimensions = Vector3f::Constant(3.0f);
		volumeConfig.resolution = Vector3i::Constant(int(3.0f / voxelSize));
		volumeConfig.translateBasis = m_trackingEngine.getTranslateBasis();
		m_pScene = volumeConfig.bIsDynamic ? new NuiKinfuOpenCLShiftScene(volumeConfig, &m_cachedPointCloud) : new NuiKinfuOpenCLScene(volumeConfig);
		break;
	}
}

void	NuiKinfuMainEngine::log(const std::string& fileName) const
{
	m_trackingEngine.log(fileName);
	if(m_pScene)
		m_pScene->log(fileName);
}

bool	NuiKinfuMainEngine::getCLData(NuiCLMappableData* pCLData, int drawMode)
{
	if(!pCLData)
		return false;

	// Camera
	pCLData->SetCameraPos( m_trackingEngine.getCameraPose() );

	// Color image
	m_trackingEngine.BufferToMappableTexture(pCLData);

	bool returnStatus = false;
	if( m_pScene )
	{
		boost::mutex::scoped_lock volumeLock(m_trackingMutex);
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

bool	NuiKinfuMainEngine::getCameraPose (NuiCameraPos* cam) const
{
	if(!cam)
		return false;

	*cam = m_trackingEngine.getCameraPose();
	/*if(m_pVolume)
		cam->setTranslation( cam->getGlobalTranslation() - m_translateBasis);*/
	return true;
}

bool	NuiKinfuMainEngine::getMesh(NuiMeshShape* pMesh)
{
	if(!m_pScene ||  !pMesh)
		return false;

	return m_pScene->Volume2Mesh(pMesh);
}

void	NuiKinfuMainEngine::setIntegrationMetricThreshold(float threshold)
{
	m_trackingEngine.setIntegrationMetricThreshold(threshold);
}

void	NuiKinfuMainEngine::resetTracker()
{
	m_trackingEngine.reset();
}

void	NuiKinfuMainEngine::resetVolume()
{
	if(m_pScene)
		m_pScene->reset();
	m_cachedPointCloud.clear();
}

void	NuiKinfuMainEngine::setTranslateBasis(const Vector3f& basis) {
	m_trackingEngine.setTranslateBasis(basis);
	m_trackingEngine.reset();
}

bool	NuiKinfuMainEngine::processFrame (
	INT64 timeStamp,
	UINT16* pDepthBuffer,
	BGRQUAD* pColorBuffer,
	UINT nWidth,
	UINT nHeight,
	const NuiCameraParams& cameraParams
	)
{
	if(!pDepthBuffer)
		return true;

	if( !m_trackingEngine.isInit() )
	{
		bool bAcceleratedFeedback = m_pScene->needAcceleratedFeedback();
		m_trackingEngine.initialize(m_trackingConfig, bAcceleratedFeedback, nWidth, nHeight);
	}

	NuiTimeLog::instance().tick(sTrackingName);
	bool returnStatus = false;
	try
	{
		boost::mutex::scoped_lock trackingLock(m_trackingMutex);
		returnStatus = m_trackingEngine.RunTracking(
			timeStamp,
			pDepthBuffer,
			pColorBuffer,
			nWidth * nHeight,
			m_pScene,
			cameraParams);
		trackingLock.unlock();
	}
	catch (std::exception& e)
	{
		LOG4CPLUS_FATAL(NuiLogger::instance().fileLogger(), e.what());
	}
	
	NuiTimeLog::instance().tock(sTrackingName);

	return returnStatus;
}

void	NuiKinfuMainEngine::CachePointCloud(NuiCLMappableData* pCLData)
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
		CL_FALSE,//blocking
		0,
		vertex_sum * 4 * sizeof(float),
		(void*)(m_cachedPointCloud.getColors() + originalSize),
		0,
		NULL,
		NULL
	);
	NUI_CHECK_CL_ERR(err);

	m_cachedPointCloud.writeUnlock();

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
