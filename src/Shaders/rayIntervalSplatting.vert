
#version 330 core

layout (location = 0) in vec4 vs_position;

uniform bool u_IsMax;
uniform vec2 u_SensorMinMax;

out float vDepth;

void main()
{
	if(u_IsMax)
	{
		gl_Position = vec4(vs_position.xy, vs_position.w, 1.0);
		vDepth = vs_position.w * (u_SensorMinMax.y - u_SensorMinMax.x) + u_SensorMinMax.x;
	}
	else
	{
		gl_Position = vec4(vs_position.xyz, 1.0);
		vDepth = vs_position.z * (u_SensorMinMax.y - u_SensorMinMax.x) + u_SensorMinMax.x;
	}
}
