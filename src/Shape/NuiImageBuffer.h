#pragma once
#include "stdafx.h"

template<typename T>
class NuiImageBuffer
{
public:
	NuiImageBuffer()
		: m_nWidth(0)
		, m_nHeight(0)
		, m_pBuffer(nullptr)
	{}
	~NuiImageBuffer()
	{
		Clear();
	}

	void			Clear()
	{
		SafeDeleteArray(m_pBuffer);
		m_nWidth = 0;
		m_nHeight = 0;
	}
	void			DeepCopy (const NuiImageBuffer& other)
	{
		if(other.m_pBuffer)
		{
			T* pBuffer = AllocateBuffer(other.m_nWidth, other.m_nHeight);
			memcpy(pBuffer, other.m_pBuffer, GetBufferSize());
		}
		else
		{
			Clear();
		}
	}
	NuiImageBuffer (const NuiImageBuffer& other){ DeepCopy(other); }
	NuiImageBuffer& operator = (const NuiImageBuffer& other) {	DeepCopy(other); return *this; }

	T*				AllocateBuffer(UINT width, UINT height)
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
				m_pBuffer = new T[m_nWidth*m_nHeight];
			}
		}
		return m_pBuffer;
	}
	T*				GetBuffer() const { return m_pBuffer; }
	UINT			GetWidth() const {	return m_nWidth; }
	UINT			GetHeight() const { return m_nHeight;	}
	UINT			GetBytesPerPixel() const { return sizeof(T); }
	UINT			GetBufferNum() const { return (m_nWidth * m_nHeight); }
	UINT			GetBufferSize() const { return (GetBytesPerPixel() * GetBufferNum()); }

private:
	T*				m_pBuffer;
	UINT			m_nWidth;
	UINT			m_nHeight;
};

// Typedefs
//
typedef NuiImageBuffer<UINT16>                   NuiDepthImage;
typedef NuiImageBuffer<BGRQUAD>                  NuiColorImage;
typedef NuiImageBuffer<BYTE>                     NuiBodyIndexImage;
typedef NuiImageBuffer<CameraSpacePoint>         NuiCameraSpaceImage;
typedef NuiImageBuffer<ColorSpacePoint>          NuiColorSpaceImage;
typedef NuiImageBuffer<DepthSpacePoint>          NuiDepthSpaceImage;