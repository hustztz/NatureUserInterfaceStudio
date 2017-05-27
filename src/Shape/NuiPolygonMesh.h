#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

#include <vector>

class NuiPolygonMesh
{
public:
	NuiPolygonMesh();
	~NuiPolygonMesh();

	void			clear();

	void			calculateMesh(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals, float leafSize);

	int				getTrianglesNum() const { return m_mesh.polygons.size(); }
	void			getVertices(pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud) const;
	void			getIndices(std::vector<uint32_t>& indices) const;
	void			save(std::string file);

private:
	pcl::PolygonMesh		m_mesh;
};
