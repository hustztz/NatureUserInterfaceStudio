#include "NuiGuiHWMappable.h"
#include "NuiGuiHWTextureMappable.h"
#include "NuiPangoRayIntervalSplattingShader.h"

#include <pangolin/pangolin.h>
#include <pangolin/gl/glsl.h>

namespace NuiPangoRayIntervalSplattingShader
{
	static pangolin::GlSlProgram sOfflineShader;
	static GLuint sFbo[1];
	static GLuint sVba[1];

	void initializeShader()
	{
		std::string shaderDir = getenv("NUI_LOCATION");
		shaderDir += "\\Shaders";

		std::map<std::string,std::string> program_defines;
		sOfflineShader.AddShaderFromFile(pangolin::GlSlVertexShader, shaderDir + "/" + "rayIntervalSplatting.vert");
		sOfflineShader.AddShaderFromFile(pangolin::GlSlFragmentShader, shaderDir + "/" + "rayIntervalSplatting.frag");
		sOfflineShader.Link();

		glGenVertexArrays(1, sVba); // Create our Vertex Array Object
	}

	void render(NuiMappable4f& vb, NuiTextureMappable& rbMin, NuiTextureMappable& rbMax, int size, float sensorDepthMin, float sensorDepthMax)
	{
		if(!vb.size() || !size)
			return;

		GLenum glError = glGetError();
		sOfflineShader.Bind();

		glDisable(GL_STENCIL_TEST);
		glEnable(GL_DEPTH_TEST);
		glDepthMask(GL_TRUE);

		// Min
		glDepthFunc(GL_LESS);
		glClear(GL_COLOR_BUFFER_BIT |GL_DEPTH_BUFFER_BIT);
		sOfflineShader.SetUniform("u_IsMax", false);
		sOfflineShader.SetUniform("u_SensorMinMax", sensorDepthMin, sensorDepthMax);

		NuiGuiHWTextureMappable::bindFrameTextureBuffer(rbMin);

		glBindVertexArray(sVba[0]);
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, NuiGuiHWMappable::asHWVertexBuffer(vb));
		glVertexAttribPointer((GLuint)0, 4, GL_FLOAT, GL_FALSE, 0, 0);

		glPushAttrib(GL_VIEWPORT_BIT);
		glDrawArrays(GL_TRIANGLES, 0, size);
		glViewport(0, 0, 512, 424);
		/*glPointSize(3.0f);
		glBegin(GL_POINTS);
		glVertex3f(0.0, 0.5, 0.0);
		glEnd();*/

		glPopAttrib();

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		//glBindRenderbuffer(GL_RENDERBUFFER_EXT, 0);
		NuiGuiHWTextureMappable::unbindFrameTextureBuffer(rbMin);

		glFlush();

		// Max
		glDepthFunc(GL_GREATER);
		glClear(GL_COLOR_BUFFER_BIT |GL_DEPTH_BUFFER_BIT);
		sOfflineShader.SetUniform("u_IsMax", true);
		sOfflineShader.SetUniform("u_SensorMinMax", sensorDepthMin, sensorDepthMax);

		//glBindRenderbuffer(GL_RENDERBUFFER_EXT, NuiGuiHWTextureMappable::asHWRenderBuffer(rbMax));
		//glFramebufferRenderbuffer(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, NuiGuiHWTextureMappable::asHWTextureBuffer(rbMax));
		NuiGuiHWTextureMappable::bindFrameTextureBuffer(rbMax);
		glBindBuffer(GL_ARRAY_BUFFER, NuiGuiHWMappable::asHWVertexBuffer(vb));

		glPushAttrib(GL_VIEWPORT_BIT);
		glDrawArrays(GL_TRIANGLES, 0, size);
		glViewport(0, 0, 512, 424);
		glPopAttrib();

		glBindVertexArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		//glBindRenderbuffer(GL_RENDERBUFFER_EXT, 0);
		//glBindFramebuffer(GL_FRAMEBUFFER_EXT,0);
		NuiGuiHWTextureMappable::unbindFrameTextureBuffer(rbMax);

		glDisable(GL_DEPTH_TEST);
		glDepthMask(GL_FALSE);
		sOfflineShader.Unbind();

		
		glFlush();
	}
}

