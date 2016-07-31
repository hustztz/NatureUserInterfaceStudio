#version 330 core

in vec4 vColor;
out vec4 FragColor;

void main()
{
	FragColor = vec4(vColor.rgb, 1.0);
}