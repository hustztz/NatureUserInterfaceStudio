#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class NuiKinfuXYZRGB;

class NuiSLAMPointCloud
{
public:
	NuiSLAMPointCloud();
	~NuiSLAMPointCloud();

	void		clear() { m_pointCloud.clear(); }

	int			pointSize() const { return (int)m_pointCloud.size(); }
	void		resizePoints(size_t size) { m_pointCloud.resize(size); }
	bool		estimateNormals(NuiKinfuXYZRGB* pXYZRGB, float filterLeafSize);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr	getPtr() { return m_pointCloud.makeShared(); }

private:
	pcl::PointCloud<pcl::PointXYZRGBNormal> m_pointCloud;
};
