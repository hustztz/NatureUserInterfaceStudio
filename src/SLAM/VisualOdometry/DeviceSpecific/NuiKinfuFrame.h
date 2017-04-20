#pragma once

#include "stdafx.h"
#include "Shape\NuiImageBuffer.h"

class NuiKinfuCameraState;

class NuiKinfuFrame
{
public:
	NuiKinfuFrame(){}
	virtual ~NuiKinfuFrame(){}

	virtual void	UpdateVertexBuffers(UINT16* pDepths, UINT* pDepthDistortionLT, UINT nNum, NuiKinfuCameraState* pCameraState) = 0;
	virtual void	UpdateColorBuffers(ColorSpacePoint* pDepthToColor, UINT* pDepthDistortionLT, UINT nNum, const NuiColorImage& image) = 0;

	virtual UINT	GetWidth() const = 0;
	virtual UINT	GetHeight() const = 0;
};