#include "NuiSLAMPointCloud.h"
#include "SLAM/VisualOdometry/NuiKinfuXYZRGB.h"
#include "Foundation/NuiLogger.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

NuiSLAMPointCloud::NuiSLAMPointCloud()
{
}

NuiSLAMPointCloud::~NuiSLAMPointCloud()
{
	clear();
}

bool	NuiSLAMPointCloud::estimateNormals(NuiKinfuXYZRGB* pXYZRGB, float filterLeafSize)
{
	if (!pXYZRGB)
		return false;

	if (filterLeafSize <= 0.0f)
		return false;

	LOG4CPLUS_INFO(NuiLogger::instance().consoleLogger(), LOG4CPLUS_TEXT("Estimating normals..."));

	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(pXYZRGB->getPtr());
	sor.setLeafSize(filterLeafSize, filterLeafSize, filterLeafSize);

	pcl::PointCloud<pcl::PointXYZRGB> tempCloud;
	sor.filter(tempCloud);

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

	tree->setInputCloud(tempCloud.makeShared());

	n.setInputCloud(tempCloud.makeShared());
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);

	pcl::concatenateFields(tempCloud, *normals, m_pointCloud);

	return true;
}
