#include "NuiSLAMPointCloudManager.h"
#include "SLAM/VisualOdometry/NuiKinfuXYZRGB.h"
#include "Shape/NuiCLMappableData.h"

using namespace NuiSLAMEngine;

NuiSLAMPointCloudManager::NuiSLAMPointCloudManager()
	: m_pVertexCache(NULL)
	, m_pCLData(NULL)
	, m_filterLeafSize(0.0f)
{
}

NuiSLAMPointCloudManager::~NuiSLAMPointCloudManager()
{
	reset();
}

/*virtual*/
void	NuiSLAMPointCloudManager::reset()
{
	m_pointCloud.clear();
}


/*virtual*/
bool	NuiSLAMPointCloudManager::process ()
{
	if(!m_pVertexCache || !m_pCLData)
	{
		//boost::this_thread::sleep (boost::posix_time::seconds (1));
		return false;
	}

	NuiKinfuXYZRGB xyzrgb;
	if (xyzrgb.incrementPoints(m_pVertexCache))
	{
		if (m_pointCloud.estimateNormals(&xyzrgb, m_filterLeafSize))
		{
			NuiPolygonMesh* pMesh = m_pCLData->GetPolygonMesh();
			if (pMesh)
			{
				xyzrgb.readLock();
				pMesh->calculateMesh(m_pointCloud.getPtr(), m_filterLeafSize);
				xyzrgb.readUnlock();
			}
		}
	}
	
	return false;
}