#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Foundation/SgVec3T.h>
#include <Foundation/SgVec4T.h>

class NuiPointCloudShape
{
public:
	NuiPointCloudShape(SgVec3f* vertices, SgVec4f* colors, int pointSize);
	~NuiPointCloudShape();

	void			clear();

	void			processCloud(float leafSize);

private:
	pcl::PointCloud<pcl::PointXYZRGB>		m_cloud;
	pcl::PointCloud<pcl::PointXYZRGBNormal> m_processedCloud;
};
