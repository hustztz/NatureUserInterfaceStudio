#pragma once

#include <pangolin/pangolin.h>

class NuiPolygonMesh;

class NuiPangoPolygonMeshShader
{
public:
	NuiPangoPolygonMeshShader();
	~NuiPangoPolygonMeshShader();

	bool initializeBuffers(NuiPolygonMesh* pMesh);
	void drawMesh(const pangolin::OpenGlMatrix& mvp, bool bNormals);
	void uninitializeBuffers();

private:
	int		m_numTriangles;
	GLuint	m_vbo;
	GLuint	m_ibo;
};