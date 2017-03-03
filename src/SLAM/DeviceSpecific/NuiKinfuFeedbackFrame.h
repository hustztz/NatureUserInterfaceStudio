#pragma once

#include "stdafx.h"

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
	};
public:
	NuiKinfuFeedbackFrame(){}
	virtual ~NuiKinfuFeedbackFrame(){}

	virtual void	UpdateBuffers(NuiKinfuFrame* frame, NuiKinfuCameraState* pCameraState) = 0;

	virtual bool	VerticesToMappablePosition(NuiCLMappableData* pMappableData) { return false; }
	virtual bool	BufferToMappableTexture(NuiCLMappableData* pMappableData, TrackerBufferType bufferType) { return false; }

	virtual UINT	GetWidth() const = 0;
	virtual UINT	GetHeight() const = 0;
protected:
	
};