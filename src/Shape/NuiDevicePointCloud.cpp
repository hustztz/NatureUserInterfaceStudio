#include "NuiDevicePointCloud.h"

NuiDevicePointCloud::NuiDevicePointCloud()
	: m_nPointNum(0)
	, m_pCloudPoints(NULL)
	, m_nWidthStep(0)
{
}

NuiDevicePointCloud::~NuiDevicePointCloud()
{
	Clear();
}

void NuiDevicePointCloud::Clear()
{
	SafeDeleteArray(m_pCloudPoints);
	m_nPointNum = 0;
	m_nWidthStep = 0;
}

void	NuiDevicePointCloud::DeepCopy (const NuiDevicePointCloud& other)
{
	m_nWidthStep = other.m_nWidthStep;

	if(other.m_pCloudPoints)
	{
		NuiDevicePoint* pBuffer = AllocatePoints(other.m_nPointNum);
		memcpy(pBuffer, other.m_pCloudPoints, m_nPointNum * sizeof(NuiDevicePoint));
	}
	else
	{
		Clear();
	}

	m_colorImage = other.m_colorImage;
}

NuiDevicePoint*	NuiDevicePointCloud::AllocatePoints(UINT num)
{
	if(m_pCloudPoints)
	{
		if(m_nPointNum != num)
		{
			Clear();
		}
	}
	if(!m_pCloudPoints)
	{
		m_nPointNum = num;
		if(m_nPointNum > 0)
		{
			m_pCloudPoints = new NuiDevicePoint[m_nPointNum];
		}
	}
	return m_pCloudPoints;
}

NuiDevicePoint*		NuiDevicePointCloud::AccessPoint(UINT id) const
{
	if(!m_pCloudPoints || id >= m_nPointNum)
		return NULL;

	return (m_pCloudPoints + id);
}

bool	NuiDevicePointCloud::ReadPoint(UINT id, NuiDevicePoint* pPt) const
{
	if(!m_pCloudPoints || !pPt || id >= m_nPointNum)
		return false;

	*pPt = *(m_pCloudPoints + id);
	return true;
}

bool	NuiDevicePointCloud::SetPoint(UINT id, const NuiDevicePoint& pt)
{
	if(!m_pCloudPoints || id >= m_nPointNum)
		return false;

	NuiDevicePoint* pPt = m_pCloudPoints + id;
	if(!pPt)
		return false;

	*pPt = pt;
	return true;
}

bool	NuiDevicePointCloud::SetPointVertex(UINT id, const Point4D& vt)
{
	if(!m_pCloudPoints || id >= m_nPointNum)
		return false;

	NuiDevicePoint* pPt = m_pCloudPoints + id;
	if(!pPt)
		return false;

	pPt->fVertex = vt;
	return true;
}
