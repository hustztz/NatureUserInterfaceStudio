
#version 330 core
#extension GL_EXT_geometry_shader4 : enable

layout(points) in;
layout(triangle_strip, max_vertices = 4) out;

uniform ivec2 u_ScreenSize;
uniform vec2 u_PointSize;

in vec4 vColor[];

out vec4 fs_color;

vec4 cQuadPts[4] = vec4[4](
		vec4( -1.0,  1.0, 0, 0 ),
		vec4( -1.0, -1.0, 0, 0 ),
		vec4(  1.0,  1.0, 0, 0 ),
		vec4(  1.0, -1.0, 0, 0 ));

void main()
{
	fs_color = vColor[0];
	
    vec4 sizeInZ = vec4(u_PointSize.xy  / u_ScreenSize.xy, 0, 0) * gl_in[0].gl_Position.w;
	for( int i = 0; i < 4; ++i ) {
		gl_Position = gl_in[0].gl_Position + sizeInZ * cQuadPts[i];
		EmitVertex();
	}
    EndPrimitive();
}
