#pragma once

#include <pangolin/pangolin.h>
#include <pangolin/gl/glsl.h>

// Forwards
class NuiCLMappableData;

class NuiPangoMeshShader
{
public:
	NuiPangoMeshShader(const std::string& shaderDir);
	~NuiPangoMeshShader();

	bool initializeBuffers(NuiCLMappableData* pData);
	void drawMesh(const pangolin::OpenGlMatrix& mvp);
	void uninitializeBuffers();

private:
	pangolin::GlSlProgram m_shader;

	UINT m_indexSize;

	GLuint m_vao;
};