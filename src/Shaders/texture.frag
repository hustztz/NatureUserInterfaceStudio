
#version 330 core

uniform sampler2D u_colorSampler;
uniform ivec2 u_TextureScale;

in vec2 vUV;
out vec4 FragColor;

void main()
{
	vec4 fColor = texture(u_colorSampler, vUV / u_TextureScale);
	FragColor = vec4(fColor.rgb, 1.0);
}
