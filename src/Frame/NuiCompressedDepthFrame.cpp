#include "NuiCompressedDepthFrame.h"

NuiCompressedDepthFrame::~NuiCompressedDepthFrame()
{
	Clear();
}

void NuiCompressedDepthFrame::Clear()
{
	SafeDeleteArray(m_pDepthBuffer);
	m_nDepthWidth = 0;
	m_nDepthHeight = 0;
}

void NuiCompressedDepthFrame::DeepCopy (const NuiCompressedDepthFrame& other)
{
	NuiGrabberFrame::DeepCopy(other);

	m_ZoomFactor = other.m_ZoomFactor;
	m_ViewOffset = other.m_ViewOffset;

	if(other.m_pDepthBuffer)
	{
		UINT16* pDepthBuffer = AllocateBuffer(other.m_nDepthWidth, other.m_nDepthHeight);
		memcpy(pDepthBuffer, other.m_pDepthBuffer, GetBufferSize());
	}
	else
	{
		Clear();
	}
}

UINT16* NuiCompressedDepthFrame::AllocateBuffer(UINT width, UINT height)
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
			m_pDepthBuffer = new UINT16[m_nDepthWidth*m_nDepthHeight];
		}
	}
	return m_pDepthBuffer;
}
