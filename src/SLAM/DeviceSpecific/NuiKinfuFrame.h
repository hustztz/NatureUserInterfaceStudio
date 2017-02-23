#pragma once

#include "stdafx.h"

class NuiColorImage;
class NuiCameraParams;

class NuiKinfuFrame
{
public:
	NuiKinfuFrame() : m_nWidth(0), m_nHeight(0) {}
	virtual ~NuiKinfuFrame()
	{
		ReleaseBuffers();
	}

	virtual void	AcquireBuffers(UINT nWidth, UINT nHeight, UINT nColorWidth, UINT nColorHeight) = 0;
	virtual void	ReleaseBuffers() = 0;
	virtual void	UpdateDepthBuffers(UINT16* pDepths, UINT nNum, float minDepth, float maxDepth) = 0;
	virtual void	UpdateColorBuffers(ColorSpacePoint* pDepthToColor, UINT nNum, const NuiColorImage& image) = 0;

	UINT	GetWidth() const { return m_nWidth; }
	UINT	GetHeight() const { return m_nHeight; }
protected:
	UINT m_nWidth, m_nHeight;
};