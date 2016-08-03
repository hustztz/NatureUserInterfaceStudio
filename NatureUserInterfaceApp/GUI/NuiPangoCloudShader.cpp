#include "NuiGuiHWMappable.h"
#include "NuiPangoCloudShader.h"

#include "Shape\NuiCLMappableData.h"

NuiPangoCloudShader::NuiPangoCloudShader(const std::string& shaderDir)
	: m_indexSize(0)
{
	std::map<std::string,std::string> program_defines;
	m_shader.AddShaderFromFile(pangolin::GlSlVertexShader, shaderDir + "/" + "draw_color.vert");
	m_shader.AddShaderFromFile(pangolin::GlSlGeometryShader, shaderDir + "/" + "draw_point_cloud.geom");
	m_shader.AddShaderFromFile(pangolin::GlSlFragmentShader, shaderDir + "/" + "color.frag");
	m_shader.Link();
}

NuiPangoCloudShader::~NuiPangoCloudShader()
{
	uninitializeBuffers();
}

bool NuiPangoCloudShader::initializeBuffers(NuiCLMappableData* pData)
{
	if(!pData)
		return false;

	std::shared_ptr<NuiVectorMappableImplui> clPointIndices =	NuiMappableAccessor::asVectorImpl(pData->PointIndices());

	if(!pData->PositionStream().size() || !pData->ColorStream().size() || !clPointIndices->data().size())
		return false;

	// ibo
	glGenBuffers(1, &m_ibo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*clPointIndices->data().size(), clPointIndices->data().data(), GL_STREAM_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	m_indexSize = (int)clPointIndices->data().size();

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

void NuiPangoCloudShader::drawPoints(const pangolin::OpenGlMatrix& mvp, float pointSize)
{
	m_shader.Bind();

	m_shader.SetUniform("u_mvp", mvp);
	m_shader.SetUniform("u_PointSize", pointSize, pointSize);
	const pangolin::Viewport& vp = pangolin::DisplayBase().GetBounds();
	m_shader.SetUniform("u_ScreenSize", vp.w, vp.h);

	glBindVertexArray(m_vao);

	glDrawElements(GL_POINTS, m_indexSize, GL_UNSIGNED_INT, 0);

	glBindVertexArray(0);

	m_shader.Unbind();
}

void NuiPangoCloudShader::uninitializeBuffers()
{
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glDeleteVertexArrays(1, &m_vao);
	glDeleteBuffers(1, &m_ibo);
}