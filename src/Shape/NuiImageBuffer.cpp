#include "NuiImageBuffer.h"

NuiImageBuffer::~NuiImageBuffer()
{
	Clear();
}

void NuiImageBuffer::Clear()
{
	SafeDeleteArray(m_pBuffer);
	m_nWidth = 0;
	m_nHeight = 0;
}

void NuiImageBuffer::DeepCopy (const NuiImageBuffer& other)
{
	if(other.m_pBuffer)
	{
		BYTE* pBuffer = AllocateBuffer(other.m_nWidth, other.m_nHeight);
		memcpy(pBuffer, other.m_pBuffer, GetBufferSize());
	}
	else
	{
		Clear();
	}
}

BYTE* NuiImageBuffer::AllocateBuffer(UINT width, UINT height)
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
			m_pBuffer = new BYTE[m_nWidth*m_nHeight];
		}
	}
	return m_pBuffer;
}