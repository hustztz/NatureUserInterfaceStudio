#include "NuiPolygonMesh.h"

#include "Foundation/NuiLogger.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <boost/make_shared.hpp>

NuiPolygonMesh::NuiPolygonMesh()
	: m_dirty(false)
	, m_numEvaluatedTriangles(0)
{

}

NuiPolygonMesh::~NuiPolygonMesh()
{
	clear();
}

void	NuiPolygonMesh::clear()
{
	m_indices.clear();
	m_cloud.clear();
	//m_mesh.cloud.clear();
	m_numEvaluatedTriangles = 0;
	setDirty();
}

void	NuiPolygonMesh::calculateMesh(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals, float leafSize)
{
	LOG4CPLUS_INFO(NuiLogger::instance().consoleLogger(), "Calculating polygon mesh...");

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
	setDirty();
	LOG4CPLUS_INFO(NuiLogger::instance().consoleLogger(), "Calculate polygon mesh completed.");
}

void	NuiPolygonMesh::save(std::string file)
{
	LOG4CPLUS_INFO(NuiLogger::instance().consoleLogger(), "Saving polygon mesh...");
	std::string filePLY = file;
	filePLY.append(".ply");
	pcl::io::savePLYFile(filePLY, m_mesh, 5);
	LOG4CPLUS_INFO(NuiLogger::instance().consoleLogger(), "Save polygon mesh completed.");
}

void	NuiPolygonMesh::evaluateMesh()
{
	int numTriangles = getTrianglesNum();
	if (m_numEvaluatedTriangles >= numTriangles)
		return;
	for (int i = m_numEvaluatedTriangles; i < numTriangles; i++)
	{
		m_indices.insert(m_indices.end(), m_mesh.polygons.at(i).vertices.begin(), m_mesh.polygons.at(i).vertices.end());
	}
	pcl::fromPCLPointCloud2(m_mesh.cloud, m_cloud);
	m_numEvaluatedTriangles = numTriangles;
}

const pcl::PointXYZRGBNormal*		NuiPolygonMesh::getVerticesBuffer() const
{
	return m_cloud.points.data();
}

UINT		NuiPolygonMesh::getVerticesBufferSize() const
{
	return m_cloud.points.size() * sizeof(pcl::PointXYZRGBNormal);
}

const uint32_t*	NuiPolygonMesh::getIndicesBuffer() const
{
	return m_indices.empty() ? NULL : &m_indices[0];
}

UINT		NuiPolygonMesh::getIndicesBufferSize() const
{
	return m_indices.size() * sizeof(uint32_t);
}