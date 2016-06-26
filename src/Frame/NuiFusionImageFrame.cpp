#include "NuiFusionImageFrame.h"

NuiFusionImageFrame::~NuiFusionImageFrame()
{
	Clear();
}

void NuiFusionImageFrame::Clear()
{
	SafeDelete(m_pBuffer);
	m_nWidth = 0;
	m_nHeight = 0;
}

void NuiFusionImageFrame::DeepCopy (const NuiFusionImageFrame& other)
{
	NuiGrabberFrame::DeepCopy(other);

	m_nFPS = other.m_nFPS;
	if(other.m_pBuffer)
	{
		BGRQUAD* pBuffer = AllocateBuffer(other.m_nWidth, other.m_nHeight);
		memcpy(pBuffer, other.m_pBuffer, GetBufferSize());
	}
	else
	{
		Clear();
	}
}

BGRQUAD* NuiFusionImageFrame::AllocateBuffer(UINT width, UINT height)
{
	if(m_pBuffer)
	{
		if(m_nWidth != width || m_nHeight != height)
		{
			Clear();
		}
	}
	if(!m_pBuffer)
	{
		m_nWidth = width;
		m_nHeight = height;
		if(0 != m_nWidth && 0 != m_nHeight)
		{
			m_pBuffer = new(std::nothrow) BGRQUAD[m_nWidth*m_nHeight];
		}
	}
	return m_pBuffer;
}