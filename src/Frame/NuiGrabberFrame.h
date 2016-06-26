#pragma once

#include "stdafx.h"

class NuiGrabberFrame
{
protected:
	static const int	sDepthFps = 30;
	static const int	sHalfADepthFrameMs = (1000 / sDepthFps) /*/ 2*/;
	static const UINT	sBodyCount = 6;

public:
	NuiGrabberFrame (INT64 timeStamp)
		: m_liTimeStamp(timeStamp)
	{
	}
	NuiGrabberFrame ()
		: m_liTimeStamp(0)
	{
	}
	virtual ~NuiGrabberFrame ()
	{}
	virtual void	Clear() = 0;
	void	DeepCopy (const NuiGrabberFrame& other)
	{
		m_liTimeStamp = other.m_liTimeStamp;
	}

	INT64	GetTimeStamp() const { return m_liTimeStamp; }
	void	SetTimeStamp(INT64 timeStamp) { m_liTimeStamp = timeStamp; }

	UINT	GetBodyCount() const { return sBodyCount; }

protected:
	INT64	m_liTimeStamp;
};