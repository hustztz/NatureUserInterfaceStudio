#include "NuiGuiHWMappable.h"
#include "NuiPangoMeshShader.h"

#include "Shape\NuiCLMappableData.h"

NuiPangoMeshShader::NuiPangoMeshShader(const std::string& shaderDir)
	: m_indexSize(0)
{
	std::map<std::string,std::string> program_defines;
	m_shader.AddShaderFromFile(pangolin::GlSlVertexShader, shaderDir + "/" + "draw_color.vert");
	m_shader.AddShaderFromFile(pangolin::GlSlFragmentShader, shaderDir + "/" + "surface.frag");
	m_shader.Link();
}

NuiPangoMeshShader::~NuiPangoMeshShader()
{
	uninitializeBuffers();
}

bool NuiPangoMeshShader::initializeBuffers(NuiCLMappableData* pData)
{
	if(!pData)
		return false;

	std::shared_ptr<NuiVectorMappableImplui> triIndices =	NuiMappableAccessor::asVectorImpl(pData->TriangleIndices());

	if(!pData->PositionStream().size() || !pData->ColorStream().size() || !triIndices->data().size())
		return false;

	// ibo
	glGenBuffers(1, &m_ibo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*triIndices->data().size(), triIndices->data().data(), GL_STREAM_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	m_indexSize = (int)triIndices->data().size();

	// vba
	glGenVertexArrays(1, &m_vao); // Create our Vertex Array Object
	glBindVertexArray(m_vao); // Bind our Vertex Array Object so we can use it

	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);

	GLuint vVertex=3;
	GLuint vColor=4;
	glBindBuffer(GL_ARRAY_BUFFER, NuiGuiHWMappable::asHWVertexBuffer(pData->PositionStream()));
	glVertexAttribPointer((GLuint)0, vVertex, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer
	glBindBuffer(GL_ARRAY_BUFFER, NuiGuiHWMappable::asHWVertexBuffer(pData->ColorStream()));
	glVertexAttribPointer((GLuint)1, vColor, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ibo);

	glBindVertexArray(0); // Disable our Vertex Buffer Object

	return true;
}

void NuiPangoMeshShader::drawMesh(const pangolin::OpenGlMatrix& mvp)
{
	m_shader.Bind();

	m_shader.SetUniform("u_mvp", mvp);

	glBindVertexArray(m_vao);

	glDrawElements(GL_TRIANGLES, m_indexSize, GL_UNSIGNED_INT, 0);

	glBindVertexArray(0);

	m_shader.Unbind();
}

void NuiPangoMeshShader::uninitializeBuffers()
{
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glDeleteVertexArrays(1, &m_vao);
	glDeleteBuffers(1, &m_ibo);
}