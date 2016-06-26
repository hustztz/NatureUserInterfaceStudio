#include "NuiFrameBuffer.h"

#include "Frame\NuiCompositeFrame.h"

NuiFrameBuffer::NuiFrameBuffer()
	: NuiFrameCacheImpl()
	, m_pFrame(NULL)
{
}

NuiFrameBuffer::~NuiFrameBuffer()
{
	clear();
}

void NuiFrameBuffer::clear()
{
	m_pFrame.reset();
}

bool	NuiFrameBuffer::pushbackFrame(std::shared_ptr<NuiCompositeFrame> pFrame)
{
	m_pFrame = pFrame;
	return true;
}

std::shared_ptr<NuiCompositeFrame>	NuiFrameBuffer::getLatestFrame()
{
	return m_pFrame;
}

std::shared_ptr<NuiCompositeFrame>	NuiFrameBuffer::getFrame(UINT frameId)
{
	return m_pFrame;
}
