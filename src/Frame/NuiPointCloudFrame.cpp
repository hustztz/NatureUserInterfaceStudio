#include "NuiPointCloudFrame.h"

NuiPointCloudFrame::NuiPointCloudFrame()
	: NuiGrabberFrame()
{
}

NuiPointCloudFrame::~NuiPointCloudFrame()
{
	Clear();
}

void NuiPointCloudFrame::Clear()
{
	m_pointCloud.Clear();
}

NuiDevicePoint*	NuiPointCloudFrame::AllocatePoints(UINT num)
{
	return m_pointCloud.AllocatePoints(num);
}

NuiDevicePoint*	NuiPointCloudFrame::AccessPoint(UINT id) const
{
	return m_pointCloud.AccessPoint(id);
}

bool	NuiPointCloudFrame::ReadPointCloud(NuiDevicePointCloud* pPointCloud) const
{
	if(!pPointCloud || (0 == m_pointCloud.GetPointsNum()))
		return false;

	*pPointCloud = m_pointCloud;
	return true;
}