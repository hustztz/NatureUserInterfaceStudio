#include "NuiGestureFrame.h"

NuiGestureFrame::NuiGestureFrame()
	: NuiGrabberFrame()
{
	for (UINT i = 0; i < sBodyCount; ++i)
	{
		m_aGestureResults[i] = nullptr;
	}
}

NuiGestureFrame::~NuiGestureFrame()
{
	Clear();
}

void NuiGestureFrame::Clear()
{
	for (UINT i = 0; i < sBodyCount; ++i)
	{
		SafeDelete(m_aGestureResults[i]);
	}
}

bool NuiGestureFrame::CacheGestureResult(UINT bodyId, NuiGestureResult* pGesture)
{
	if(bodyId >= sBodyCount)
		return false;

	if(pGesture == m_aGestureResults[bodyId])
		return true;

	SafeDelete(m_aGestureResults[bodyId]);
	if(pGesture)
		m_aGestureResults[bodyId] = pGesture;
	return true;
}

bool NuiGestureFrame::ReadGestureResult(UINT bodyId, NuiGestureResult* pGesture) const
{
	if(!pGesture)
		return false;

	if(bodyId >= sBodyCount)
		return false;

	if(!m_aGestureResults[bodyId])
		return false;

	*pGesture = *m_aGestureResults[bodyId];
	return true;
}