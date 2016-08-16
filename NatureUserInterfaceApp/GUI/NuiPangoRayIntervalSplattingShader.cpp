#include "NuiGuiHWMappable.h"
#include "NuiPangoRayIntervalSplattingShader.h"

#include <pangolin/pangolin.h>
#include <pangolin/gl/glsl.h>

namespace NuiPangoRayIntervalSplattingShader
{
	static pangolin::GlSlProgram sOfflineShader;

	void initializeShader(const std::string& shaderDir)
	{
		std::map<std::string,std::string> program_defines;
		sOfflineShader.AddShaderFromFile(pangolin::GlSlVertexShader, shaderDir + "/" + "rayIntervalSplatting.vert");
		sOfflineShader.AddShaderFromFile(pangolin::GlSlFragmentShader, shaderDir + "/" + "rayIntervalSplatting.frag");
		sOfflineShader.Link();
	}

	void render(NuiMappable4f& vb, int size, UINT16 sensorDepthMin, UINT16 sensorDepthMax)
	{
		if(!vb.size() || !size)
			return;

		sOfflineShader.Bind();
		// Min
		sOfflineShader.SetUniform("u_IsMax", (int)0);
		sOfflineShader.SetUniform("u_SensorMinMax", (int)sensorDepthMin, (int)sensorDepthMax);

		glBindBuffer(GL_ARRAY_BUFFER, NuiGuiHWMappable::asHWVertexBuffer(vb));

		glDrawArrays(GL_TRIANGLES, 0, size);

		glBindBuffer(GL_ARRAY_BUFFER, 0);

		// Max
		sOfflineShader.SetUniform("u_IsMax", (int)1);
		sOfflineShader.SetUniform("u_SensorMinMax", (int)sensorDepthMin, (int)sensorDepthMax);

		glBindBuffer(GL_ARRAY_BUFFER, NuiGuiHWMappable::asHWVertexBuffer(vb));

		glDrawArrays(GL_TRIANGLES, 0, size);

		glBindBuffer(GL_ARRAY_BUFFER, 0);

		sOfflineShader.Unbind();
	}
}

