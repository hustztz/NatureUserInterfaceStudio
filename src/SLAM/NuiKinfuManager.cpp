#include "NuiKinfuManager.h"

#include "NuiKinfuTSDFVolume.h"
#include "NuiHashingVolume.h"
#include "Shape/NuiCLMappableData.h"
#include "Foundation/NuiTimeLog.h"

static const std::string sTrackerName("Tracker");

NuiKinfuManager::NuiKinfuManager()
	: m_bAutoReset(false)
	, m_pVolume(NULL)
	, m_translateBasis(Vector3f::Zero())
{
	
}

NuiKinfuManager::~NuiKinfuManager()
{
	stopThread();
	SafeDelete(m_pVolume);
}

void	NuiKinfuManager::resetVolume(float voxelSize, bool bHashingSDF)
{
	SafeDelete(m_pVolume);

	if(bHashingSDF)
	{
		NuiHashingSDFConfig sdfConfig;
		sdfConfig.m_virtualVoxelSize = voxelSize;
		sdfConfig.m_truncation = 5.0f * sdfConfig.m_virtualVoxelSize;
		sdfConfig.m_truncScale = 2.5f * sdfConfig.m_virtualVoxelSize;
		NuiHashingRaycastConfig raycastConfig;
		m_pVolume = new NuiHashingVolume(sdfConfig, raycastConfig);
	}
	else
	{
		NuiKinfuVolumeConfig volumeConfig;
		volumeConfig.dimensions = Vector3f::Constant(3.0f);
		volumeConfig.resolution = Vector3i::Constant(int(3.0f / voxelSize));
		m_pVolume = new NuiKinfuTSDFVolume(volumeConfig);
	}
}

void	NuiKinfuManager::log(const std::string& fileName) const
{
	m_tracker.log(fileName);
	if(m_pVolume)
		m_pVolume->log(fileName);
}

bool	NuiKinfuManager::getCLData(NuiCLMappableData* pCLData, bool bIsMesh)
{
	if(!pCLData)
		return false;

	// Camera
	pCLData->SetCameraParams( m_tracker.getCameraPose() );

	// Color image
	//m_tracker.previousNormalImageToData(pCLData);

	//bool returnStatus = m_tracker.previousBufferToData(pCLData);
	bool returnStatus = false;
	if( m_pVolume )
	{
		returnStatus = bIsMesh ? m_pVolume->Volume2CLMesh(pCLData) : m_pVolume->Volume2CLVertices(pCLData);
	}

	return returnStatus;
}

bool	NuiKinfuManager::getCameraPose (NuiCameraPos* cam) const
{
	if(!cam)
		return false;

	*cam = m_tracker.getCameraPose();
	/*if(m_pVolume)
		cam->setTranslation( cam->getTranslation() - m_translateBasis);*/
	return true;
}

bool	NuiKinfuManager::getMesh(NuiMeshShape* pMesh)
{
	if(!m_pVolume ||  !pMesh)
		return false;

	return m_pVolume->Volume2Mesh(pMesh);
}

void	NuiKinfuManager::setIntegrationMetricThreshold(float threshold)
{
	m_tracker.setIntegrationMetricThreshold(threshold);
}

/*virtual*/
void	NuiKinfuManager::reset()
{
	m_buffer.clear();

	m_tracker.reset(m_translateBasis);
	if(m_pVolume)
		m_pVolume->reset();
}

/*virtual*/
bool	NuiKinfuManager::process ()
{
	std::shared_ptr<NuiCompositeFrame> pCompositeFrame = m_buffer.popCompositeFrame();
	if(!pCompositeFrame)
	{
		//boost::this_thread::sleep (boost::posix_time::seconds (1));
		return true;
	}

	const UINT nPointWidth = pCompositeFrame->m_depthFrame.GetWidth();
	const UINT nPointHeight = pCompositeFrame->m_depthFrame.GetHeight();
	const UINT nPointNum = nPointWidth * nPointHeight;
	UINT16* pDepthBuffer = pCompositeFrame->m_depthFrame.GetBuffer();
	if(!pDepthBuffer)
		return true;

	const UINT16 minDepth = pCompositeFrame->m_depthFrame.GetMinDepth();
	const UINT16 maxDepth = pCompositeFrame->m_depthFrame.GetMaxDepth();

	const UINT nColorMapWidth = pCompositeFrame->m_colorMapFrame.GetWidth();
	const UINT nColorMapHeight = pCompositeFrame->m_colorMapFrame.GetHeight();
	const UINT nColorMapNum = nColorMapWidth * nColorMapHeight;
	ColorSpacePoint* pDepthToColor = pCompositeFrame->m_colorMapFrame.GetBuffer();
	//assert(nPointNum == nColorMapNum);

	const NuiCameraIntrinsics& intri = pCompositeFrame->GetCameraParams().getIntrinsics();

	if( !m_tracker.isInit() )
	{
		const UINT nImageWidth = pCompositeFrame->m_colorFrame.GetWidth();
		const UINT nImageHeight = pCompositeFrame->m_colorFrame.GetHeight();
		m_tracker.initialize(m_trackerConfig, nPointWidth, nPointHeight, nImageWidth, nImageHeight);
	}

	NuiTimeLog::instance().tick(sTrackerName);
	bool returnStatus = m_tracker.RunTracking(
		pDepthBuffer,
		nPointNum,
		minDepth,
		maxDepth,
		pDepthToColor,
		pCompositeFrame->m_colorFrame.GetImage(),
		m_pVolume,
		intri.m_fx, intri.m_fy, intri.m_cx, intri.m_cy);
	NuiTimeLog::instance().tock(sTrackerName);

	pCompositeFrame.reset();

	if(!returnStatus)
	{
		if(m_bAutoReset)
		{
			m_tracker.reset(m_translateBasis);
			/*if(m_tsdf_volume)
				m_tsdf_volume->reset();*/
		}
		else
		{
			std::cout << "Fusion quite." << std::endl;
			return false;
		}
	}

	return true;
}