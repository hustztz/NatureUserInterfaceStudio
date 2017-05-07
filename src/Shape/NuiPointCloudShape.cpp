#include "NuiPointCloudShape.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

#include <boost/make_shared.hpp>

NuiPointCloudShape::NuiPointCloudShape(SgVec3f* vertices, SgVec4f* colors, int pointSize)
{
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int i = 0; i < pointSize; i++)
	{
		pcl::PointXYZRGB pt;
		pt.x = vertices[i][0];
		pt.y = vertices[i][1];
		pt.z = vertices[i][2];
		pt.r = colors[i][0];
		pt.g = colors[i][1];
		pt.b = colors[i][2];
		pt.a = colors[i][3];
		m_cloud.push_back(pt);
	}
}

NuiPointCloudShape::~NuiPointCloudShape()
{
	clear();
}

void	NuiPointCloudShape::clear() {
	m_cloud.clear();
	m_processedCloud.clear();
}

void	NuiPointCloudShape::processCloud(float leafSize)
{
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(m_cloud));
	sor.setLeafSize(leafSize, leafSize, leafSize);

	pcl::PointCloud<pcl::PointXYZRGB> tempCloud;
	sor.filter(tempCloud);



	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal> normals;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

	tree->setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(tempCloud));

	n.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(tempCloud));
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(normals);

	pcl::concatenateFields(tempCloud, normals, m_processedCloud);
}
