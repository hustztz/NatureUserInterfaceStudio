#pragma once

#include <pangolin/pangolin.h>
#include <pangolin/gl/glsl.h>

// Forwards
class NuiCLMappableData;

class NuiPangoTexturedCloudShader
{
public:
	NuiPangoTexturedCloudShader(const std::string& shaderDir);
	~NuiPangoTexturedCloudShader();

	bool initializeBuffers(NuiCLMappableData* pData);
	void drawPoints(const pangolin::OpenGlMatrix& mvp, GLuint textureId, float pointSize);
	void uninitializeBuffers();

private:
	pangolin::GlSlProgram m_shader;

	UINT m_indexSize;
	int m_textureWidth;
	int m_textureHeight;

	GLuint m_vao;
};