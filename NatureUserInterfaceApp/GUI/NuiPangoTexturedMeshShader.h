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
	void drawMesh(const pangolin::OpenGlMatrix& mvp);
	void uninitializeBuffers();

private:
	pangolin::GlSlProgram m_shader;

	UINT m_indexSize;
	int m_textureWidth;
	int m_textureHeight;
	GLuint m_texId;
	GLuint m_vao;
};