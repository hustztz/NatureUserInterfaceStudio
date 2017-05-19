#include "NuiKinfuManager.h"
#include "Shape\NuiCameraParams.h"

NuiKinfuManager::NuiKinfuManager()
	: m_bAutoReset(false)
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
	std::shared_ptr<NuiVisualFrame> pVisualFrame = m_buffer.popVisualFrame();
	if(!pVisualFrame)
	{
		//boost::this_thread::sleep (boost::posix_time::seconds (1));
		return true;
	}

	bool bSucceed = m_engine.processFrame(
		pVisualFrame->getTimeStamp(),
		pVisualFrame->getDepthBuffer(),
		pVisualFrame->getColorBuffer(),
		pVisualFrame->getWidth(),
		pVisualFrame->getHeight(),
		pVisualFrame->getCameraParams()
	);

	pVisualFrame.reset();

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
	return true;
}