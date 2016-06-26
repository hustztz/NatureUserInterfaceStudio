#include "NuiSkeletonFrame.h"

NuiSkeletonFrame::NuiSkeletonFrame()
	: NuiGrabberFrame()
{
	for (UINT i = 0; i < sBodyCount; ++i)
	{
		m_aSkeletons[i] = nullptr;
	}
}

NuiSkeletonFrame::~NuiSkeletonFrame()
{
	Clear();
}

void NuiSkeletonFrame::Clear()
{
	for (UINT i = 0; i < sBodyCount; ++i)
	{
		SafeDelete(m_aSkeletons[i]);
	}
}

bool NuiSkeletonFrame::CacheSkeleton(UINT skeletonId, NuiSkeletonJoints* pSkeleton)
{
	if(skeletonId >= sBodyCount)
		return false;

	if(pSkeleton == m_aSkeletons[skeletonId])
		return true;

	SafeDelete(m_aSkeletons[skeletonId]);
	if(pSkeleton)
		m_aSkeletons[skeletonId] = pSkeleton;
	return true;
}

bool NuiSkeletonFrame::ReadSkeleton(UINT skeletonId, NuiSkeletonJoints* pSkeleton) const
{
	if(!pSkeleton)
		return false;

	if(skeletonId >= sBodyCount)
		return false;

	if(!m_aSkeletons[skeletonId])
		return false;

	*pSkeleton = *m_aSkeletons[skeletonId];
	return true;
}
