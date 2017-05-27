#include "NuiPolygonMesh.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <boost/make_shared.hpp>

NuiPolygonMesh::NuiPolygonMesh()
{

}

NuiPolygonMesh::~NuiPolygonMesh()
{
	clear();
}

void	NuiPolygonMesh::clear()
{

}

void	NuiPolygonMesh::calculateMesh(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals, float leafSize)
{
	// Create search tree*
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(leafSize * 2.0);

	// Set typical values for the parameters
	gp3.setMu(2);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 2); // 45 degrees
	gp3.setMinimumAngle(M_PI / 36); // 10 degrees
	gp3.setMaximumAngle(2.5 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);

	gp3.reconstruct(m_mesh);
}

void	NuiPolygonMesh::getVertices(pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud) const
{
	pcl::fromPCLPointCloud2(m_mesh.cloud, cloud);
}

void	NuiPolygonMesh::getIndices(std::vector<uint32_t>& indices) const
{
	int numTriangles = getTrianglesNum();
	for (int i = 0; i < numTriangles; i++)
	{
		indices.insert(indices.end(), m_mesh.polygons.at(i).vertices.begin(), m_mesh.polygons.at(i).vertices.end());
	}
}

void	NuiPolygonMesh::save(std::string file)
{
	std::string filePLY = file;
	filePLY.append(".ply");
	pcl::io::savePLYFile(filePLY, m_mesh, 5);
}