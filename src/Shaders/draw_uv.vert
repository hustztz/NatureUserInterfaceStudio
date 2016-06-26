
#version 330 core

layout (location = 0) in vec3 vs_position;
layout (location = 1) in vec2 vs_texCoord;

uniform mat4 u_mvp;

out vec2 vUV;

void main()
{
	if(vs_position.z > 0.0)
	{
		gl_Position = u_mvp*vec4(vs_position, 1.0);
		vUV = vs_texCoord;
	}
	else
	{
		gl_Position = vec4(0.0, 0.0, 0.0, 0.0);
		vUV = vec2(0.0, 0.0);
	}
}
