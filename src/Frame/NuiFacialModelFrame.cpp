#include "NuiFacialModelFrame.h"

NuiFacialModelFrame::NuiFacialModelFrame()
	: NuiGrabberFrame()
{
	for (UINT i = 0; i < sBodyCount; ++i)
	{
		m_facialModel[i] = nullptr;
	}
}

NuiFacialModelFrame::~NuiFacialModelFrame()
{
	Clear();
}

void NuiFacialModelFrame::Clear()
{
	for (UINT i = 0; i < sBodyCount; ++i)
	{
		SafeDelete(m_facialModel[i]);
	}
}

bool NuiFacialModelFrame::CacheFacialModel(UINT faceId, NuiFacialModel* pFace)
{
	if(faceId >= sBodyCount)
		return false;

	if(pFace == m_facialModel[faceId])
		return true;

	SafeDelete(m_facialModel[faceId]);
	if(pFace)
		m_facialModel[faceId] = pFace;
	return true;
}

bool	NuiFacialModelFrame::ReadFacialModel(UINT faceId, NuiFacialModel* pFace) const
{
	if(!pFace)
		return false;

	if(faceId >= sBodyCount)
		return false;

	if(!m_facialModel[faceId])
		return false;

	*pFace = *m_facialModel[faceId];
	return true;
}