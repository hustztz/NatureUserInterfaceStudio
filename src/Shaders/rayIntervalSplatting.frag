#version 330 core

in vec vDepth;
out vec FragDepth;

void main()
{
	FragDepth = vDepth;
}