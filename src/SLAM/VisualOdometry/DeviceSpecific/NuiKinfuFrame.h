#pragma once

#include "stdafx.h"
#include "Shape\NuiImageBuffer.h"

#include <atomic>

class NuiKinfuCameraState;
class NuiCLMappableData;

class NuiKinfuFrame
{
public:
	NuiKinfuFrame(){}
	virtual ~NuiKinfuFrame(){}

	void setDirty() { m_dirty = true; }
	void clearDirty() { m_dirty = false; }

	virtual void	UpdateVertexBuffers(UINT16* pDepths, UINT* pDepthDistortionLT, UINT nNum, NuiKinfuCameraState* pCameraState) = 0;
	virtual void	UpdateColorBuffers(ColorSpacePoint* pDepthToColor, UINT* pDepthDistortionLT, UINT nNum, const NuiColorImage& image) = 0;
	virtual bool	BufferToMappableTexture(NuiCLMappableData* pMappableData) { return false; }

	virtual UINT	GetWidth() const = 0;
	virtual UINT	GetHeight() const = 0;
protected:
	std::atomic<bool>	m_dirty;
};