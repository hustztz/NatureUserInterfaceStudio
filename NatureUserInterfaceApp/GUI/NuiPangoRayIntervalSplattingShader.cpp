#include "NuiGuiHWMappable.h"
#include "NuiGuiHWTextureMappable.h"
#include "NuiPangoRayIntervalSplattingShader.h"

#include <pangolin/pangolin.h>
#include <pangolin/gl/glsl.h>

namespace NuiPangoRayIntervalSplattingShader
{
	static pangolin::GlSlProgram sOfflineShader;
	static GLuint sFbo[1];

	void initializeShader()
	{
		std::string shaderDir = getenv("NUI_LOCATION");
		shaderDir += "\\Shaders";

		std::map<std::string,std::string> program_defines;
		sOfflineShader.AddShaderFromFile(pangolin::GlSlVertexShader, shaderDir + "/" + "rayIntervalSplatting.vert");
		sOfflineShader.AddShaderFromFile(pangolin::GlSlFragmentShader, shaderDir + "/" + "rayIntervalSplatting.frag");
		sOfflineShader.Link();

		glGenFramebuffersEXT(1, sFbo);
	}

	void render(NuiMappable4f& vb, NuiTextureMappable& rbMin, NuiTextureMappable& rbMax, int size, float sensorDepthMin, float sensorDepthMax)
	{
		if(!vb.size() || !size)
			return;

		sOfflineShader.Bind();

		glDisable(GL_STENCIL_TEST);
		glEnable(GL_DEPTH_TEST);
		glDepthMask(GL_TRUE);

		// Min
		glDepthFunc(GL_LESS);
		glClear(GL_COLOR_BUFFER_BIT |GL_DEPTH_BUFFER_BIT);
		sOfflineShader.SetUniform("u_IsMax", false);
		sOfflineShader.SetUniform("u_SensorMinMax", sensorDepthMin, sensorDepthMax);

		glEnable(GL_FRAMEBUFFER_EXT);
		glBindFramebuffer(GL_FRAMEBUFFER_EXT, sFbo[0]);
		//glBindRenderbuffer(GL_RENDERBUFFER_EXT, NuiGuiHWTextureMappable::asHWRenderBuffer(rbMin));
		glFramebufferRenderbuffer(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER_EXT, NuiGuiHWTextureMappable::asHWRenderBuffer(rbMin));

		glBindBuffer(GL_ARRAY_BUFFER, NuiGuiHWMappable::asHWVertexBuffer(vb));

		glDrawArrays(GL_TRIANGLES, 0, size);
		glViewport(0, 0, 512, 424);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		//glBindRenderbuffer(GL_RENDERBUFFER_EXT, 0);
		glBindFramebuffer(GL_FRAMEBUFFER_EXT,0);  
		glDisable(GL_FRAMEBUFFER_EXT);

		// Max
		glDepthFunc(GL_GREATER);
		glClear(GL_COLOR_BUFFER_BIT |GL_DEPTH_BUFFER_BIT);
		sOfflineShader.SetUniform("u_IsMax", true);
		sOfflineShader.SetUniform("u_SensorMinMax", sensorDepthMin, sensorDepthMax);

		glBindRenderbuffer(GL_RENDERBUFFER_EXT, NuiGuiHWTextureMappable::asHWRenderBuffer(rbMax));

		glBindBuffer(GL_ARRAY_BUFFER, NuiGuiHWMappable::asHWVertexBuffer(vb));

		glDrawArrays(GL_TRIANGLES, 0, size);
		glViewport(0, 0, 512, 424);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindRenderbuffer(GL_RENDERBUFFER_EXT, 0);

		glDisable(GL_DEPTH_TEST);
		glDepthMask(GL_FALSE);
		sOfflineShader.Unbind();
	}
}

