
#version 330 core

in vec4 fs_color;
out vec4 FragColor;

void main()
{
	FragColor = vec4(fs_color.rgb, 1.0);
}
