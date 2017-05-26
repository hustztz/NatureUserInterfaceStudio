#include "NuiKinfuTrackingManager.h"
#include "DeviceSpecific/NuiKinfuScene.h"
#include "Shape\NuiCameraParams.h"
#include "Shape/NuiCLMappableData.h"
#include "Foundation/NuiTimeLog.h"
#include "Foundation/NuiLogger.h"

using namespace NuiKinfuEngine;

static const std::string sTrackingName("TrackingEngine");

NuiKinfuTrackingManager::NuiKinfuTrackingManager()
	: m_bAutoReset(false)
	, m_pScene(NULL)
{
}

NuiKinfuTrackingManager::~NuiKinfuTrackingManager()
{
	reset();
}

/*virtual*/
void	NuiKinfuTrackingManager::reset()
{
	m_buffer.clear();
	m_trackingEngine.reset();
}

void	NuiKinfuTrackingManager::evaluateCLData(NuiCLMappableData* pCLData)
{
	if (!pCLData)
		return;

	// Camera
	pCLData->SetCameraPos(m_trackingEngine.getCameraPose());

	// Color image
	m_trackingEngine.BufferToMappableTexture(pCLData);
}

void	NuiKinfuTrackingManager::setIntegrationMetricThreshold(float threshold)
{
	m_trackingEngine.setIntegrationMetricThreshold(threshold);
}

void	NuiKinfuTrackingManager::setTranslateBasis(const Vector3f& basis) {
	m_trackingEngine.setTranslateBasis(basis);
	m_trackingEngine.reset();
}

/*virtual*/
bool	NuiKinfuTrackingManager::process ()
{
	std::shared_ptr<NuiVisualFrame> pVisualFrame = m_buffer.popVisualFrame();
	if(!pVisualFrame)
	{
		//boost::this_thread::sleep (boost::posix_time::seconds (1));
		return true;
	}

	const UINT nWidth = pVisualFrame->getWidth();
	const UINT nHeight = pVisualFrame->getHeight();

	if (!m_trackingEngine.isInit())
	{
		bool bAcceleratedFeedback = m_pScene ? m_pScene->needAcceleratedFeedback() : false;
		m_trackingEngine.initialize(m_trackingConfig, bAcceleratedFeedback, nWidth, nHeight);
	}

	NuiTimeLog::instance().tick(sTrackingName);
	bool bSucceed = false;
	try
	{
		boost::mutex::scoped_lock trackingLock(m_trackingMutex);
		bSucceed = m_trackingEngine.RunTracking(
			pVisualFrame->getTimeStamp(),
			pVisualFrame->getDepthBuffer(),
			pVisualFrame->getColorBuffer(),
			nWidth * nHeight,
			m_pScene,
			pVisualFrame->getCameraParams());
		trackingLock.unlock();
	}
	catch (std::exception& e)
	{
		LOG4CPLUS_FATAL(NuiLogger::instance().fileLogger(), e.what());
	}

	NuiTimeLog::instance().tock(sTrackingName);

	pVisualFrame.reset();

	if( !bSucceed )
	{
		if(m_bAutoReset)
		{
			m_trackingEngine.reset();
			LOG4CPLUS_INFO(NuiLogger::instance().consoleLogger(), "Tracking reset.");
		}
		else
		{
			LOG4CPLUS_INFO(NuiLogger::instance().consoleLogger(), "Tracking quite.");
			return false;
		}
	}
	return true;
}