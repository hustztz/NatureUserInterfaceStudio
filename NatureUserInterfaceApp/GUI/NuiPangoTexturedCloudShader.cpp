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
	if(!pData)
		return false;

	std::shared_ptr<NuiVectorMappableImpl3f> positions = NuiMappableAccessor::asVectorImpl(pData->PositionStream());
	std::shared_ptr<NuiVectorMappableImpl2f> uvs = NuiMappableAccessor::asVectorImpl(pData->PatchUVStream());
	std::shared_ptr<NuiVectorMappableImplui> clPointIndices =	NuiMappableAccessor::asVectorImpl(pData->PointIndices());

	if(!positions->data().size() || !uvs->data().size() || !clPointIndices->data().size())
		return false;

	// vbo
	glGenBuffers(2, m_vbos); // Generate our Vertex Buffer Object
	glBindBuffer(GL_ARRAY_BUFFER, m_vbos[0]); // Bind our Vertex Buffer Object
	glBufferData(GL_ARRAY_BUFFER, sizeof(SgVec3f)*positions->data().size(), positions->data().data(), GL_STREAM_DRAW); // Set the size and data of our VBO and set it to GL_STREAM_DRAW

	glBindBuffer(GL_ARRAY_BUFFER, m_vbos[1]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(SgVec2f)*uvs->data().size(), uvs->data().data(), GL_STREAM_DRAW);

	m_textureWidth = pData->GetColorImage().GetWidth();
	m_textureHeight = pData->GetColorImage().GetHeight();

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
	GLuint vUV=2;
	glBindBuffer(GL_ARRAY_BUFFER, m_vbos[0]);
	glVertexAttribPointer((GLuint)0, vVertex, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer
	glBindBuffer(GL_ARRAY_BUFFER, m_vbos[1]);
	glVertexAttribPointer((GLuint)1, vUV, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ibo);

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

	glDeleteVertexArrays(1, &m_vao);
	glDeleteBuffers(1, &m_vbos[0]);
	glDeleteBuffers(1, &m_vbos[1]);
	glDeleteBuffers(1, &m_ibo);
}