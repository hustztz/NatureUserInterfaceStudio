#include "NuiCompoundImageFrame.h"

NuiCompoundImageFrame::NuiCompoundImageFrame()
	: NuiGrabberFrame()
{
}

NuiCompoundImageFrame::~NuiCompoundImageFrame()
{
	Clear();
}

void NuiCompoundImageFrame::Clear()
{
	m_compoundImage.Clear();
}

NuiCompoundPixel*	NuiCompoundImageFrame::AllocatePixels(UINT w, UINT h)
{
	return m_compoundImage.AllocatePixels(w, h);
}

bool NuiCompoundImageFrame::ReadPixel(UINT w, UINT h, NuiCompoundPixel* pPixel) const
{
	if(!pPixel)
		return false;

	return m_compoundImage.ReadPixel(w, h, pPixel);
}