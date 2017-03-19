#include "NuiKinfuManager.h"
#include "Shape\NuiCameraParams.h"

NuiKinfuManager::NuiKinfuManager()
	: m_bAutoReset(false)
	, m_bIsStepIn(false)
{
}

NuiKinfuManager::~NuiKinfuManager()
{
}

/*virtual*/
void	NuiKinfuManager::reset()
{
	m_buffer.clear();
	m_engine.resetTracker();
	m_engine.resetVolume();
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

	const UINT nWidth = pCompositeFrame->m_depthFrame.GetWidth();
	const UINT nHeight = pCompositeFrame->m_depthFrame.GetHeight();
	const UINT nPointNum = nWidth * nHeight;
	UINT16* pDepthBuffer = pCompositeFrame->m_depthFrame.GetBuffer();
	
	const UINT nColorMapWidth = pCompositeFrame->m_colorMapFrame.GetWidth();
	const UINT nColorMapHeight = pCompositeFrame->m_colorMapFrame.GetHeight();
	const UINT nColorMapNum = nColorMapWidth * nColorMapHeight;
	ColorSpacePoint* pDepthToColor = pCompositeFrame->m_colorMapFrame.GetBuffer();
	//assert(nPointNum == nColorMapNum);

	NuiCameraParams cameraParams;
	cameraParams.m_intrinsics = pCompositeFrame->GetCameraParams().getIntrinsics();
	cameraParams.m_sensorDepthMax = (float)(pCompositeFrame->m_depthFrame.GetMaxDepth()) / 1000.0f;
	cameraParams.m_sensorDepthMin = (float)(pCompositeFrame->m_depthFrame.GetMinDepth()) / 1000.0f;

	bool bSucceed = m_engine.processFrame(
		pDepthBuffer,
		pDepthToColor,
		nWidth,
		nHeight,
		pCompositeFrame->m_colorFrame.GetImage(),
		cameraParams);

	pCompositeFrame.reset();

	if( !bSucceed )
	{
		if(m_bAutoReset)
		{
			m_engine.resetTracker();
			std::cout << "Fusion reset." << std::endl;
		}
		else
		{
			std::cout << "Fusion quite." << std::endl;
			return false;
		}
	}

	if(m_bIsStepIn)
		m_threadPause = true;

	return true;
}