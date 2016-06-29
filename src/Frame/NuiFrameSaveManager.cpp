#include "NuiFrameSaveManager.h"

NuiFrameSaveManager::NuiFrameSaveManager(const std::string&	fileName)
	: m_fileName(fileName)
	, m_frameId(0)
{
	
}

NuiFrameSaveManager::~NuiFrameSaveManager()
{
	stopThread();
}

/*virtual*/
void	NuiFrameSaveManager::reset()
{
	m_buffer.clear();
	m_frameId = 0;
}

/*virtual*/
bool	NuiFrameSaveManager::process ()
{
	std::shared_ptr<NuiCompositeFrame> pCompositeFrame = m_buffer.popCompositeFrame();
	if(!pCompositeFrame)
	{
		//boost::this_thread::sleep (boost::posix_time::seconds (1));
		return true;
	}

	bool bSaved = false;

	std::stringstream ssTime;
	ssTime << m_frameId;
	std::string timeStamp;
	ssTime >> timeStamp;

	std::string imageFileName = m_fileName;
	imageFileName.append("\\");
	imageFileName.append(timeStamp);
	imageFileName.append(".depth");
	if( pCompositeFrame->m_depthFrame.saveFrame(imageFileName) )
		bSaved = true;

	imageFileName = m_fileName;
	imageFileName.append("\\");
	imageFileName.append(timeStamp);
	imageFileName.append(".color");
	if( pCompositeFrame->m_colorFrame.saveFrame(imageFileName) )
		bSaved = true;

	imageFileName = m_fileName;
	imageFileName.append("\\");
	imageFileName.append(timeStamp);
	imageFileName.append(".cmap");
	if( pCompositeFrame->m_colorMapFrame.saveFrame(imageFileName) )
		bSaved = true;

	imageFileName = m_fileName;
	imageFileName.append("\\");
	imageFileName.append(timeStamp);
	imageFileName.append(".vertex");
	if( pCompositeFrame->m_cameraMapFrame.saveFrame(imageFileName) )
		bSaved = true;

	imageFileName = m_fileName;
	imageFileName.append("\\");
	imageFileName.append(timeStamp);
	imageFileName.append(".camIntri");
	if( pCompositeFrame->m_cameraParams.save(imageFileName) )
		bSaved = true;

	if(bSaved)
		m_frameId ++;

	pCompositeFrame.reset();
	return true;
}