#pragma once

#include <pangolin/pangolin.h>
#include <pangolin/gl/glsl.h>

// Forwards
class NuiCLMappableData;

class NuiPangoTexturedMeshShader
{
public:
	NuiPangoTexturedMeshShader(const std::string& shaderDir);
	~NuiPangoTexturedMeshShader();

	bool initializeBuffers(NuiCLMappableData* pData);
	void drawMesh(const pangolin::OpenGlMatrix& mvp, GLuint textureId);
	void uninitializeBuffers();

private:
	pangolin::GlSlProgram m_shader;

	UINT m_indexSize;
	int m_textureWidth;
	int m_textureHeight;

	GLuint m_vao;
	GLuint m_vbos[2];
	GLuint m_ibo;
};