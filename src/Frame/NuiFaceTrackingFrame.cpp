#include "NuiFaceTrackingFrame.h"

NuiFaceTrackingFrame::NuiFaceTrackingFrame()
	: NuiGrabberFrame()
{
	for (UINT i = 0; i < sBodyCount; ++i)
	{
		m_aTrackedFace[i] = nullptr;
	}
}

NuiFaceTrackingFrame::~NuiFaceTrackingFrame()
{
	Clear();
}

void NuiFaceTrackingFrame::Clear()
{
	for (UINT i = 0; i < sBodyCount; ++i)
	{
		SafeDelete(m_aTrackedFace[i]);
	}
}

bool NuiFaceTrackingFrame::CacheTrackedFace(UINT faceId, NuiTrackedFace* pFace)
{
	if(faceId >= sBodyCount)
		return false;

	if(pFace == m_aTrackedFace[faceId])
		return true;

	SafeDelete(m_aTrackedFace[faceId]);
	if(pFace)
		m_aTrackedFace[faceId] = pFace;
	return true;
}

bool	NuiFaceTrackingFrame::ReadTrackedFace(UINT faceId, NuiTrackedFace* pFace) const
{
	if(!pFace)
		return false;

	if(faceId >= sBodyCount)
		return false;

	if(!m_aTrackedFace[faceId])
		return false;

	*pFace = *m_aTrackedFace[faceId];
	return true;
}