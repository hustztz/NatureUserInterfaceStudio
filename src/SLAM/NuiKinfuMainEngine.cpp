#include "NuiKinfuMainEngine.h"

#include "DeviceSpecific/OpenCL/NuiKinfuOpenCLScene.h"
#include "Shape/NuiCLMappableData.h"
#include "Foundation/NuiTimeLog.h"

static const std::string sTrackingName("TrackingEngine");

using namespace NuiKinfuEngine;

NuiKinfuMainEngine::NuiKinfuMainEngine()
	: m_pScene(NULL)
	, m_translateBasis(Vector3f::Zero())
{
}

NuiKinfuMainEngine::~NuiKinfuMainEngine()
{
	SafeDelete(m_pScene);
}


void	NuiKinfuMainEngine::setVolume(float voxelSize, bool bHashingSDF)
{
	SafeDelete(m_pScene);

	/*if(bHashingSDF)
	{
		NuiHashingSDFConfig sdfConfig;
		sdfConfig.m_virtualVoxelSize = voxelSize;
		sdfConfig.m_truncation = 5.0f * sdfConfig.m_virtualVoxelSize;
		sdfConfig.m_truncScale = 2.5f * sdfConfig.m_virtualVoxelSize;
		NuiHashingRaycastConfig raycastConfig;
		m_pScene = new NuiHashingVolume(sdfConfig, raycastConfig);
	}
	else*/
	{
		NuiKinfuVolumeConfig volumeConfig;
		volumeConfig.dimensions = Vector3f::Constant(3.0f);
		volumeConfig.resolution = Vector3i::Constant(int(3.0f / voxelSize));
		m_pScene = new NuiKinfuOpenCLScene(volumeConfig);
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
	pCLData->SetCameraParams( m_trackingEngine.getCameraPose() );

	// Color image
	m_trackingEngine.BufferToMappableTexture(pCLData);

	//bool returnStatus = m_tracker.previousBufferToData(pCLData);
	bool returnStatus = false;
	if( m_pScene )
	{
		returnStatus = bIsMesh ? m_pScene->Volume2CLMesh(pCLData) : m_pScene->Volume2CLVertices(pCLData);
	}

	return returnStatus;
}

bool	NuiKinfuMainEngine::getCameraPose (NuiCameraPos* cam) const
{
	if(!cam)
		return false;

	*cam = m_trackingEngine.getCameraPose();
	/*if(m_pVolume)
		cam->setTranslation( cam->getTranslation() - m_translateBasis);*/
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

void	NuiKinfuMainEngine::offlineRender()
{
	if(m_pScene)
		m_pScene->offlineRender();
}

void	NuiKinfuMainEngine::resetTracker()
{
	m_trackingEngine.reset(m_translateBasis);
}

void	NuiKinfuMainEngine::resetVolume()
{
	if(m_pScene)
		m_pScene->reset();
}

bool	NuiKinfuMainEngine::processFrame (
	UINT16* pDepthBuffer,
	ColorSpacePoint* pDepthToColor,
	UINT nWidth,
	UINT nHeight,
	const NuiColorImage& image,
	const NuiCameraParams& cameraParams
	)
{
	if(!pDepthBuffer)
		return true;

	if( !m_trackingEngine.isInit() )
	{
		m_trackingEngine.initialize(m_trackingConfig, nWidth, nHeight, image.GetWidth(), image.GetHeight());
	}

	NuiTimeLog::instance().tick(sTrackingName);
	bool returnStatus = m_trackingEngine.RunTracking(
		pDepthBuffer,
		pDepthToColor,
		nWidth * nHeight,
		image,
		m_pScene,
		cameraParams);
	NuiTimeLog::instance().tock(sTrackingName);

	return returnStatus;
}
