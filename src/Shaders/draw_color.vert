
#version 330 core

layout (location = 0) in vec3 vs_position;
layout (location = 1) in vec4 vs_color;

uniform mat4 u_mvp;

out vec4 vColor;

void main()
{
	if(vs_position.z > 0.0)
	{
		gl_Position = u_mvp*vec4(vs_position, 1.0);
		vColor = vs_color;
	}
	else
	{
		gl_Position = vec4(0.0, 0.0, 0.0, 0.0);
		vColor = vec4(0.0, 0.0, 0.0, 0.0);
	}
}
