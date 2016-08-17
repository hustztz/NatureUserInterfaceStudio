#version 330 core

in float vDepth;
out float FragDepth;

void main()
{
	FragDepth = vDepth;
}