#pragma once

#include "NuiGrabberFrame.h"
#include "Shape/NuiSkeletonJoints.h"

class NuiSkeletonFrame : public NuiGrabberFrame
{
public:
	NuiSkeletonFrame();
	virtual ~NuiSkeletonFrame();

	virtual void	Clear() override;
	bool			CacheSkeleton(UINT skeletonId, NuiSkeletonJoints* pSkeleton);
	bool			ReadSkeleton(UINT skeletonId, NuiSkeletonJoints* pSkeleton) const;

private:
	// Disable
	NuiSkeletonFrame (const NuiSkeletonFrame& other);
	NuiSkeletonFrame& operator = (const NuiSkeletonFrame& other);
private:
	NuiSkeletonJoints*	m_aSkeletons[sBodyCount];
};