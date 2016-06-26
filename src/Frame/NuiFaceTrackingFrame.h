#pragma once

#include "NuiGrabberFrame.h"
#include "Shape/NuiTrackedFace.h"

class NuiFaceTrackingFrame : public NuiGrabberFrame
{
public:
	NuiFaceTrackingFrame();
	virtual ~NuiFaceTrackingFrame();

	virtual void	Clear() override;
	bool			CacheTrackedFace(UINT faceId, NuiTrackedFace* pFace);
	bool			ReadTrackedFace(UINT faceId, NuiTrackedFace* pFace) const;

private:
	// Disable
	NuiFaceTrackingFrame (const NuiFaceTrackingFrame& other);
	NuiFaceTrackingFrame& operator = (const NuiFaceTrackingFrame& other);
private:
	NuiTrackedFace*		m_aTrackedFace[sBodyCount];
};