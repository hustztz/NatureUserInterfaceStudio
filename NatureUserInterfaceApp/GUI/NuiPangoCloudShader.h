#pragma once

#include <pangolin/pangolin.h>
#include <pangolin/gl/glsl.h>

// Forwards
class NuiCLMappableData;

class NuiPangoCloudShader
{
public:
	NuiPangoCloudShader(const std::string& shaderDir);
	~NuiPangoCloudShader();

	bool initializeBuffers(NuiCLMappableData* pData);
	void drawPoints(const pangolin::OpenGlMatrix& mvp, float pointSize);
	void uninitializeBuffers();

private:
	pangolin::GlSlProgram m_shader;

	UINT m_indexSize;

	GLuint m_vao;
	GLuint m_ibo;
};