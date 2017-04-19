#pragma once

#include "stdafx.h"

#include <atomic>

class NuiKinfuFrame;
class NuiKinfuCameraState;
class NuiCLMappableData;

class NuiKinfuFeedbackFrame
{
public:
	enum TrackerBufferType
	{
		eTracker_Vertices = 0,
		eTracker_Normals,
		eTracker_Ranges,
	};
public:
	NuiKinfuFeedbackFrame(){}
	virtual ~NuiKinfuFeedbackFrame(){}

	void setDirty() { m_dirty = true; }
	void clearDirty() { m_dirty = false; }

	virtual void	UpdateBuffers(NuiKinfuFrame* frame, NuiKinfuCameraState* pCameraState) = 0;

	virtual bool	VerticesToMappablePosition(NuiCLMappableData* pMappableData) { return false; }
	virtual bool	BufferToMappableTexture(NuiCLMappableData* pMappableData, TrackerBufferType bufferType) { return false; }

	virtual UINT	GetWidth() const = 0;
	virtual UINT	GetHeight() const = 0;
protected:
	std::atomic<bool>	m_dirty;
};