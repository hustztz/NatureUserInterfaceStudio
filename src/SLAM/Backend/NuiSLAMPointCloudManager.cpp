#include "NuiSLAMPointCloudManager.h"
#include "SLAM/VisualOdometry/NuiKinfuXYZRGB.h"

using namespace NuiSLAMEngine;

NuiSLAMPointCloudManager::NuiSLAMPointCloudManager()
	: m_pVertexCache(NULL)
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
	if(!m_pVertexCache)
	{
		//boost::this_thread::sleep (boost::posix_time::seconds (1));
		return true;
	}

	NuiKinfuXYZRGB xyzrgb;
	if (xyzrgb.incrementPoints(m_pVertexCache))
	{
		if (m_pointCloud.estimateNormals(&xyzrgb, m_filterLeafSize))
		{
			xyzrgb.readLock();
			m_mesh.calculateMesh(m_pointCloud.getPtr(), m_filterLeafSize);
			xyzrgb.readUnlock();
		}
	}
	
	return true;
}