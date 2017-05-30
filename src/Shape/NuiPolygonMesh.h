#pragma once

#include "stdafx.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

#include <atomic>
#include <vector>

class NuiPolygonMesh
{
public:
	NuiPolygonMesh();
	~NuiPolygonMesh();

	void			clear();

	void			setDirty() { m_dirty = true; }
	void			clearDirty() { m_dirty = false; }
	bool			isDirty() const { return m_dirty; }

	void			calculateMesh(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals, float leafSize);
	void			save(std::string file);

	int				getTrianglesNum() const { return m_mesh.polygons.size(); }
	void			evaluateMesh();
	const pcl::PointXYZRGBNormal*	getVerticesBuffer() const;
	UINT			getVerticesBufferSize() const;
	const uint32_t*	getIndicesBuffer() const;
	UINT			getIndicesBufferSize() const;

private:
	pcl::PolygonMesh		m_mesh;

	std::vector<uint32_t>					m_indices;
	int										m_numEvaluatedTriangles;
	pcl::PointCloud<pcl::PointXYZRGBNormal> m_cloud;
	std::atomic<bool>						m_dirty;
};
