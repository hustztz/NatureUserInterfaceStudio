#pragma once
#include "NuiMemoryBlock.h"

#include <Eigen/Geometry>

template<typename T>
class NuiImageBuffer : public NuiMemoryBlock <T>
{
public:
	NuiImageBuffer()
		: m_nWidth(0)
		, m_nHeight(0)
		, NuiMemoryBlock()
	{}
	~NuiImageBuffer()
	{
		Clear();
	}

	void			Clear()
	{
		NuiMemoryBlock::Clear();
		m_nWidth = 0;
		m_nHeight = 0;
	}
	NuiImageBuffer (const NuiImageBuffer& other){ DeepCopy(other); }
	NuiImageBuffer& operator = (const NuiImageBuffer& other) {	DeepCopy(other); return *this; }

	T*				AllocateBuffer(UINT width, UINT height)
	{
		if(m_nWidth != width || m_nHeight != height)
		{
			Clear();
		}
		m_nWidth = width;
		m_nHeight = height;
		return NuiMemoryBlock::AllocateBuffer(m_nWidth*m_nHeight);
	}
	UINT			GetWidth() const {	return m_nWidth; }
	UINT			GetHeight() const { return m_nHeight;	}
protected:
	void			DeepCopy (const NuiImageBuffer& other)
	{
		NuiMemoryBlock::DeepCopy(other);
		m_nWidth = other.GetWidth();
		m_nHeight = other.GetHeight();
	}
private:
	UINT			m_nWidth;
	UINT			m_nHeight;
};

// Typedefs
//
typedef NuiImageBuffer<UINT16>                   NuiDepthImage;
typedef NuiImageBuffer<BGRQUAD>                  NuiColorImage;
typedef NuiImageBuffer<BYTE>                     NuiBodyIndexImage;
typedef NuiImageBuffer<float>                    NuiFloatImage;
typedef NuiImageBuffer<CameraSpacePoint>         NuiCameraSpaceImage;
typedef NuiImageBuffer<ColorSpacePoint>          NuiColorSpaceImage;
typedef NuiImageBuffer<DepthSpacePoint>          NuiDepthSpaceImage;
typedef NuiImageBuffer<Eigen::Vector3f>			 NuiFloat3Image;