#version 330 core
#extension GL_EXT_geometry_shader4 : enable

layout(points) in;
layout(points, max_vertices = 1) out;

in vec4 vColor[];

out vec4 fs_color;

void main()
{
	fs_color = vColor[0];
    gl_Position = gl_in[0].gl_Position;

	EmitVertex();
    EndPrimitive();
}