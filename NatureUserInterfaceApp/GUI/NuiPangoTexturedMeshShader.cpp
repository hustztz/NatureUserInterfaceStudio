#include "NuiGuiHWMappable.h"
#include "NuiGuiHWTextureMappable.h"
#include "NuiPangoTexturedMeshShader.h"

#include "Shape\NuiCLMappableData.h"

NuiPangoTexturedMeshShader::NuiPangoTexturedMeshShader(const std::string& shaderDir)
	: m_indexSize(0)
	, m_textureWidth(0)
	, m_textureHeight(0)
{
	std::map<std::string,std::string> program_defines;
	m_shader.AddShaderFromFile(pangolin::GlSlVertexShader, shaderDir + "/" + "draw_uv.vert");
	m_shader.AddShaderFromFile(pangolin::GlSlFragmentShader, shaderDir + "/" + "texture.frag");
	m_shader.Link();
}

NuiPangoTexturedMeshShader::~NuiPangoTexturedMeshShader()
{
	uninitializeBuffers();
}

bool NuiPangoTexturedMeshShader::initializeBuffers(NuiCLMappableData* pData)
{
	if(!pData || !pData->PositionStream().size() || !pData->PatchUVStream().size() || !pData->TriangleIndices().size())
		return false;

	m_textureWidth = pData->ColorTex().width();
	m_textureHeight = pData->ColorTex().height();
	m_texId = NuiGuiHWTextureMappable::asHWTextureBuffer(pData->ColorTex());
	m_indexSize = (UINT)pData->TriangleIndices().size();

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
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, NuiGuiHWMappable::asHWIndexBuffer(pData->TriangleIndices()));

	glBindVertexArray(0); // Disable our Vertex Buffer Object

	return true;
}

void NuiPangoTexturedMeshShader::drawMesh(const pangolin::OpenGlMatrix& mvp)
{
	m_shader.Bind();

	m_shader.SetUniform("u_mvp", mvp);
	if(m_textureWidth > 0 && m_textureHeight > 0)
		m_shader.SetUniform("u_TextureScale", m_textureWidth, m_textureHeight);
	m_shader.SetUniform("u_colorSampler", 0);

	glEnable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, m_texId);

	glBindVertexArray(m_vao);

	glDrawElements(GL_TRIANGLES, m_indexSize, GL_UNSIGNED_INT, 0);

	glBindVertexArray(0);

	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);
	
	m_shader.Unbind();
}

void NuiPangoTexturedMeshShader::uninitializeBuffers()
{
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glDeleteVertexArrays(1, &m_vao);
}