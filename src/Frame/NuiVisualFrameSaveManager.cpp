#include "NuiVisualFrameSaveManager.h"

NuiVisualFrameSaveManager::NuiVisualFrameSaveManager(const std::string&	fileName)
	: m_fileName(fileName)
	, m_bCompressed(true)
	, m_frameId(0)
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
	m_frameId = 0;
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

	std::stringstream ssTime;
	ssTime << m_frameId;
	std::string timeStamp;
	ssTime >> timeStamp;

	std::string imageFileName = m_fileName;
	imageFileName.append("\\");
	imageFileName.append(timeStamp);

	bool bSaved = pVisualFrame->saveFrame(imageFileName, m_bCompressed);
	if (bSaved)
	{
		m_frameId++;
	}

	pVisualFrame.reset();
	return bSaved;
}