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

	int currentNumTriangles = pMesh->getTrianglesNum();
	if (abs(currentNumTriangles - m_numTriangles) > 100)
	{
		pMesh->evaluateMesh();
		m_numTriangles = currentNumTriangles;
	}
	if (0 == m_numTriangles)
		return false;

	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glBufferData(GL_ARRAY_BUFFER, pMesh->getVerticesBufferSize(), pMesh->getVerticesBuffer(), GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, pMesh->getIndicesBufferSize(), pMesh->getIndicesBuffer(), GL_STATIC_DRAW);
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