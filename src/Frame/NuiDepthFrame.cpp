#include "NuiDepthFrame.h"

NuiDepthFrame::~NuiDepthFrame()
{
	Clear();
}

void NuiDepthFrame::Clear()
{
	m_depthImage.Clear();
}

void NuiDepthFrame::DeepCopy (const NuiDepthFrame& other)
{
	NuiGrabberFrame::DeepCopy(other);

	m_nDepthFPS = other.m_nDepthFPS;
	m_depthImage = other.m_depthImage;
}
