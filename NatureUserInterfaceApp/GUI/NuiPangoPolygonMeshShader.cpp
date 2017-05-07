#include "NuiPangoPolygonMeshShader.h"

#include "Shape\NuiPolygonMesh.h"

NuiPangoPolygonMeshShader::NuiPangoPolygonMeshShader()
	: m_numTriangles(0)
{
	glGenBuffers(1, &m_vbo);
	glGenBuffers(1, &m_ibo);
}

NuiPangoPolygonMeshShader::~NuiPangoPolygonMeshShader()
{
	uninitializeBuffers();
}

bool NuiPangoPolygonMeshShader::initializeBuffers(NuiPolygonMesh* pMesh)
{
	if(!pMesh)
		return false;

	m_numTriangles = pMesh->getTrianglesNum();
	std::vector<uint32_t> indices;
	pMesh->getIndices(indices);
	pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
	pMesh->getVertices(cloud);

	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glBufferData(GL_ARRAY_BUFFER, cloud.points.size() * sizeof(pcl::PointXYZRGBNormal), cloud.points.data(), GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(uint32_t), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	return true;
}

void NuiPangoPolygonMeshShader::drawMesh(const pangolin::OpenGlMatrix& mvp, bool bNormals)
{
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glVertexPointer(3, GL_FLOAT, sizeof(pcl::PointXYZRGBNormal), 0);

	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	if (bNormals)
	{
		glColorPointer(3, GL_FLOAT, sizeof(pcl::PointXYZRGBNormal), (void *)(sizeof(float) * 4));
	}
	else
	{
		glColorPointer(3, GL_UNSIGNED_BYTE, sizeof(pcl::PointXYZRGBNormal), (void *)(sizeof(float) * 8));
	}

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ibo);

	glDrawElements(GL_TRIANGLES, m_numTriangles * 3, GL_UNSIGNED_INT, 0);

	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void NuiPangoPolygonMeshShader::uninitializeBuffers()
{
	glDeleteBuffers(1, &m_vbo);
	glDeleteBuffers(1, &m_ibo);
}