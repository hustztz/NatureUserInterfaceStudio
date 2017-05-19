#include "NuiKinfuMainEngine.h"

#include "DeviceSpecific/OpenCL/NuiKinfuOpenCLShiftScene.h"
#include "DeviceSpecific/OpenCL/NuiKinfuOpenCLHashScene.h"
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


void	NuiKinfuMainEngine::setVolume(float voxelSize, bool bHashingSDF)
{
	SafeDelete(m_pScene);

	if(bHashingSDF)
	{
		NuiHashingSDFConfig sdfConfig;
		sdfConfig.m_bUseSwapping = false;
		sdfConfig.m_bUseForwardRender = true;
		sdfConfig.m_virtualVoxelSize = voxelSize;
		//sdfConfig.m_truncation = 5.0f * sdfConfig.m_virtualVoxelSize;
		//sdfConfig.m_truncScale = 2.5f * sdfConfig.m_virtualVoxelSize;
		//NuiHashingRaycastConfig raycastConfig;
		m_pScene = new NuiKinfuOpenCLHashScene(sdfConfig);
	}
	else
	{
		NuiKinfuVolumeConfig volumeConfig;
		volumeConfig.bIsDynamic = false;
		volumeConfig.dimensions = Vector3f::Constant(3.0f);
		volumeConfig.resolution = Vector3i::Constant(int(3.0f / voxelSize));
		volumeConfig.translateBasis = m_trackingEngine.getTranslateBasis();
		m_pScene = volumeConfig.bIsDynamic ? new NuiKinfuOpenCLShiftScene(volumeConfig) : new NuiKinfuOpenCLScene(volumeConfig);
	}
}

void	NuiKinfuMainEngine::log(const std::string& fileName) const
{
	m_trackingEngine.log(fileName);
	if(m_pScene)
		m_pScene->log(fileName);
}

bool	NuiKinfuMainEngine::getCLData(NuiCLMappableData* pCLData, bool bIsMesh)
{
	if(!pCLData)
		return false;

	// Camera
	pCLData->SetCameraPos( m_trackingEngine.getCameraPose() );

	// Color image
	m_trackingEngine.BufferToMappableTexture(pCLData);

	bool returnStatus = false;
	//returnStatus = m_trackingEngine.VerticesToMappablePosition(pCLData);
	if( m_pScene )
	{
		boost::mutex::scoped_lock volumeLock(m_trackingMutex);
		returnStatus = bIsMesh ? m_pScene->Volume2CLMesh(pCLData) : m_pScene->Volume2CLVertices(pCLData);
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
