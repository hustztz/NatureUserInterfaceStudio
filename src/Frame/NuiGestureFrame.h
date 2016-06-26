#pragma once

#include "NuiGrabberFrame.h"
#include "Shape/NuiGestureResult.h"

class NuiGestureFrame : public NuiGrabberFrame
{
public:
	NuiGestureFrame();
	virtual ~NuiGestureFrame();

	virtual void	Clear() override;
	bool			CacheGestureResult(UINT bodyId, NuiGestureResult* pGesture);
	bool			ReadGestureResult(UINT bodyId, NuiGestureResult* pGesture) const;

private:
	// Disable
	NuiGestureFrame (const NuiGestureFrame& other);
	NuiGestureFrame& operator = (const NuiGestureFrame& other);
private:
	NuiGestureResult*	m_aGestureResults[sBodyCount];
};