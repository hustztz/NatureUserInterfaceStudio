#include "NuiExtendedDepthFrame.h"

NuiExtendedDepthFrame::~NuiExtendedDepthFrame()
{
	Clear();
}

void NuiExtendedDepthFrame::Clear()
{
	SafeDeleteArray(m_pDepthBuffer);
	m_nDepthWidth = 0;
	m_nDepthHeight = 0;
}

void NuiExtendedDepthFrame::DeepCopy (const NuiExtendedDepthFrame& other)
{
	NuiGrabberFrame::DeepCopy(other);

	if(other.m_pDepthBuffer)
	{
		NUI_DEPTH_IMAGE_PIXEL* pDepthBuffer = AllocateBuffer(other.m_nDepthWidth, other.m_nDepthHeight);
		memcpy(pDepthBuffer, other.m_pDepthBuffer, GetBufferSize());
	}
	else
	{
		Clear();
	}
}

NUI_DEPTH_IMAGE_PIXEL* NuiExtendedDepthFrame::AllocateBuffer(UINT width, UINT height)
{
	if(m_pDepthBuffer)
	{
		if(m_nDepthWidth != width || m_nDepthHeight != height)
		{
			Clear();
		}
	}
	if(!m_pDepthBuffer)
	{
		m_nDepthWidth = width;
		m_nDepthHeight = height;
		if(0 != m_nDepthWidth && 0 != m_nDepthHeight)
		{
			m_pDepthBuffer = new NUI_DEPTH_IMAGE_PIXEL[m_nDepthWidth*m_nDepthHeight];
		}
	}
	return m_pDepthBuffer;
}