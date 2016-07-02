#include "NuiKinfuManager.h"

#include "NuiKinfuVolume.h"
#include "Shape/NuiCLMappableData.h"
#include "Foundation/NuiTimeLog.h"

static const std::string sTrackerName("Tracker");

NuiKinfuManager::NuiKinfuManager()
	: m_bAutoReset(false)
	, m_tsdf_volume(NULL)
	, m_translateBasis(Vector3f::Zero())
{
	
}

NuiKinfuManager::~NuiKinfuManager()
{
	stopThread();
	SafeDelete(m_tsdf_volume);
}

void	NuiKinfuManager::resetVolume()
{
	SafeDelete(m_tsdf_volume);
	m_tsdf_volume = new NuiKinfuVolume(m_volumeConfig);
}

bool	NuiKinfuManager::getCLData(NuiCLMappableData* pCLData, bool bIsMesh)
{
	if(!pCLData)
		return false;

	// Camera
	NuiCameraPos pos = m_tracker.getCameraPose();
	if(m_tsdf_volume)
		pos.setTranslation( pos.getTranslation() - m_tsdf_volume->getDimensions() / 2.0f);
	pCLData->SetCameraParams(pos);

	// Color image
	pCLData->SetColorImage(m_frameColorImage);

	//bool returnStatus = m_pTracker->PreviousBuffer(pCLData);
	bool returnStatus = false;
	if( m_tsdf_volume )
	{
		returnStatus = bIsMesh ? m_tsdf_volume->Volume2CLMesh(pCLData) : m_tsdf_volume->Volume2CLVertices(pCLData);
	}

	return returnStatus;
}

bool	NuiKinfuManager::getCameraPose (NuiCameraPos* cam) const
{
	if(!cam)
		return false;

	*cam = m_tracker.getCameraPose();
	if(m_tsdf_volume)
		cam->setTranslation( cam->getTranslation() - m_translateBasis - m_tsdf_volume->getDimensions() * 0.5f);
	return true;
}

bool	NuiKinfuManager::getMesh(NuiMeshShape* pMesh)
{
	if(!m_tsdf_volume ||  !pMesh)
		return false;

	return m_tsdf_volume->Volume2Mesh(pMesh);
}

void	NuiKinfuManager::setIntegrationMetricThreshold(float threshold)
{
	if(m_tsdf_volume)
		m_tsdf_volume->setIntegrationMetricThreshold(threshold);
}

/*virtual*/
void	NuiKinfuManager::reset()
{
	m_buffer.clear();

	Vector3f volumeBasis = m_translateBasis;
	if(m_tsdf_volume)
	{
		const Vector3f& volumeDimensions = m_tsdf_volume->getDimensions();
		volumeBasis = volumeBasis + volumeDimensions * 0.5f;
	}
	m_tracker.reset(volumeBasis);
	if(m_tsdf_volume)
		m_tsdf_volume->reset();
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

	m_frameColorImage = pCompositeFrame->m_colorFrame.GetImage();
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
		m_frameColorImage,
		m_tsdf_volume,
		intri.m_fx, intri.m_fy, intri.m_cx, intri.m_cy);
	NuiTimeLog::instance().tock(sTrackerName);

	pCompositeFrame.reset();

	if(!returnStatus)
	{
		if(m_bAutoReset)
		{
			Vector3f volumeBasis = m_translateBasis;
			if(m_tsdf_volume)
			{
				const Vector3f& volumeDimensions = m_tsdf_volume->getDimensions();
				volumeBasis = volumeBasis + volumeDimensions * 0.5f;
			}
			m_tracker.reset(volumeBasis);
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