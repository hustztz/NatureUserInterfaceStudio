#include "NuiFrameLoader.h"

#include "../NuiRGBDDeviceBufferImpl.h"
#include "Frame/NuiCompositeFrame.h"

NuiFrameLoader::NuiFrameLoader()
	: m_pBuffer(NULL)
	, m_frameId(0)
{

}

NuiFrameLoader::~NuiFrameLoader()
{
	stopThread();
}

void NuiFrameLoader::reset()
{
	m_frameId = 0;
	if(m_pBuffer)
		m_pBuffer->clear();
}

/*virtual*/
bool NuiFrameLoader::process()
{
	if(!m_pBuffer)
		return false;

	std::shared_ptr<NuiCompositeFrame> pCompositeFrame = m_pBuffer->allocateFrame();
	if(!pCompositeFrame)
	{
		//assert(pCompositeFrame);
		return true;
	}

	bool bLoadSucceed = false;

	std::stringstream ssTime;
	ssTime << m_frameId;
	std::string timeStamp;
	ssTime >> timeStamp;

	std::string imageFileName = m_fileName;
	imageFileName.append("\\");
	imageFileName.append(timeStamp);
	imageFileName.append(".depth");
	if( pCompositeFrame->m_depthFrame.loadFrame(imageFileName) )
		bLoadSucceed = true;

	imageFileName = m_fileName;
	imageFileName.append("\\");
	imageFileName.append(timeStamp);
	imageFileName.append(".color");
	if( pCompositeFrame->m_colorFrame.loadFrame(imageFileName) )
		bLoadSucceed = true;

	imageFileName = m_fileName;
	imageFileName.append("\\");
	imageFileName.append(timeStamp);
	imageFileName.append(".cmap");
	if( pCompositeFrame->m_colorMapFrame.loadFrame(imageFileName) )
		bLoadSucceed = true;

	imageFileName = m_fileName;
	imageFileName.append("\\");
	imageFileName.append(timeStamp);
	imageFileName.append(".vertex");
	if( pCompositeFrame->m_cameraMapFrame.loadFrame(imageFileName) )
		bLoadSucceed = true;

	imageFileName = m_fileName;
	imageFileName.append("\\");
	imageFileName.append(timeStamp);
	imageFileName.append(".camIntri");
	if( pCompositeFrame->m_cameraParams.load(imageFileName) )
		bLoadSucceed = true;

	if(bLoadSucceed)
	{
		m_frameId ++;
	}
	else
	{
		pCompositeFrame = m_pBuffer->popFrame();
		pCompositeFrame.reset();
		return false;
	}
	
	return true;
}