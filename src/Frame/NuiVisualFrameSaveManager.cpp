#include "NuiVisualFrameSaveManager.h"

NuiVisualFrameSaveManager::NuiVisualFrameSaveManager(const std::string&	fileName)
	: m_fileName(fileName)
	, m_bCompressed(true)
{
	
}

NuiVisualFrameSaveManager::~NuiVisualFrameSaveManager()
{
	stopThread();
}

/*virtual*/
void	NuiVisualFrameSaveManager::reset()
{
	m_buffer.clear();
}

/*virtual*/
bool	NuiVisualFrameSaveManager::process ()
{
	std::shared_ptr<NuiVisualFrame> pVisualFrame = m_buffer.popVisualFrame();
	if(!pVisualFrame)
	{
		//boost::this_thread::sleep (boost::posix_time::seconds (1));
		return true;
	}

	bool bSaved = pVisualFrame->saveFrame(m_fileName, m_bCompressed);

	pVisualFrame.reset();
	return bSaved;
}