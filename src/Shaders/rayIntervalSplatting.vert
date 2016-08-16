
#version 330 core

layout (location = 0) in vec4 vs_position;

uniform ivec u_IsMax;
uniform ivec2 u_SensorMinMax;

out vec vDepth;

void main()
{
	if(u_IsMax)
	{
		gl_Position = vec4(vs_position.xy, vs_position.w, 1.0);
		vDepth = vs_position.w * (vs_position.y - vs_position.x) + vs_position.x;
	}
	else
	{
		gl_Position = vec4(vs_position, 1.0);
		vDepth = vs_position.z * (vs_position.y - vs_position.x) + vs_position.x;
	}
}
