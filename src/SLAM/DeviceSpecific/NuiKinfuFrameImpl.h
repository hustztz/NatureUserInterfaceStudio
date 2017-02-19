#pragma once

#include "stdafx.h"

class NuiColorImage;
class NuiCameraParams;

class NuiKinfuFrameImpl
{
public:
	NuiKinfuFrameImpl(){}
	virtual ~NuiKinfuFrameImpl()
	{
		ReleaseBuffers();
	}

	virtual void	AcquireBuffers(UINT nWidth, UINT nHeight, UINT nColorWidth, UINT nColorHeight) = 0;
	virtual void	ReleaseBuffers() = 0;
	virtual void	UpdateDepthBuffers(UINT16* pDepths, UINT nNum, float minDepth, float maxDepth) = 0;
	virtual void	UpdateColorBuffers(ColorSpacePoint* pDepthToColor, UINT nNum, const NuiColorImage& image) = 0;
	virtual void	UpdateCameraParams(const NuiCameraParams& camParams) = 0;
};