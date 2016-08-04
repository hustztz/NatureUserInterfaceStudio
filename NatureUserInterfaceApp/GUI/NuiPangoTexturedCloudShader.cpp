#include "NuiGuiHWMappable.h"
#include "NuiPangoTexturedCloudShader.h"
#include "Shape\NuiCLMappableData.h"

NuiPangoTexturedCloudShader::NuiPangoTexturedCloudShader(const std::string& shaderDir)
	: m_indexSize(0)
	, m_textureWidth(0)
	, m_textureHeight(0)
{
	std::map<std::string,std::string> program_defines;
	m_shader.AddShaderFromFile(pangolin::GlSlVertexShader, shaderDir + "/" + "draw_texture.vert");
	m_shader.AddShaderFromFile(pangolin::GlSlGeometryShader, shaderDir + "/" + "draw_point_cloud.geom");
	m_shader.AddShaderFromFile(pangolin::GlSlFragmentShader, shaderDir + "/" + "color.frag");
	m_shader.Link();
}

NuiPangoTexturedCloudShader::~NuiPangoTexturedCloudShader()
{
	uninitializeBuffers();
}

bool NuiPangoTexturedCloudShader::initializeBuffers(NuiCLMappableData* pData)
{
	if(!pData || !pData->PositionStream().size() || !pData->PatchUVStream().size() || !pData->PointIndices().size())
		return false;

	m_textureWidth = pData->GetColorImage().GetWidth();
	m_textureHeight = pData->GetColorImage().GetHeight();
	m_indexSize = (UINT)pData->PointIndices().size();

	// vba
	glGenVertexArrays(1, &m_vao); // Create our Vertex Array Object
	glBindVertexArray(m_vao); // Bind our Vertex Array Object so we can use it

	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);

	GLuint vVertex=3;
	GLuint vUV=2;
	glBindBuffer(GL_ARRAY_BUFFER, NuiGuiHWMappable::asHWVertexBuffer(pData->PositionStream()));
	glVertexAttribPointer((GLuint)0, vVertex, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer
	glBindBuffer(GL_ARRAY_BUFFER, NuiGuiHWMappable::asHWVertexBuffer(pData->PatchUVStream()));
	glVertexAttribPointer((GLuint)1, vUV, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, NuiGuiHWMappable::asHWIndexBuffer(pData->PointIndices()));

	glBindVertexArray(0); // Disable our Vertex Buffer Object

	return true;
}

void NuiPangoTexturedCloudShader::drawPoints(const pangolin::OpenGlMatrix& mvp, GLuint textureId, float pointSize)
{
	m_shader.Bind();

	m_shader.SetUniform("u_mvp", mvp);
	m_shader.SetUniform("u_PointSize", pointSize, pointSize);
	const pangolin::Viewport& vp = pangolin::DisplayBase().GetBounds();
	m_shader.SetUniform("u_ScreenSize", vp.w, vp.h);
	if(m_textureWidth > 0 && m_textureHeight > 0)
		m_shader.SetUniform("u_TextureScale", m_textureWidth, m_textureHeight);
	m_shader.SetUniform("u_colorSampler", 0);

	glEnable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureId);

	glBindVertexArray(m_vao);

	glDrawElements(GL_POINTS, m_indexSize, GL_UNSIGNED_INT, 0);

	glBindVertexArray(0);

	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);
	
	m_shader.Unbind();
}

void NuiPangoTexturedCloudShader::uninitializeBuffers()
{
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glDeleteVertexArrays(1, &m_vao);
}