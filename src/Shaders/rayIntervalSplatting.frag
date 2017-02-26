#version 330 core

in float vDepth;
out vec4 FragColor;

void main()
{
	/*if(vDepth > 0.0f && vDepth < 100.0f)
	{
		float seg0 = floor(vDepth);
		float remain = vDepth - seg0;
		seg0 = seg0 / 100.0f;
		remain = remain * 100.0f;
		float seg1 = floor(remain);
		remain = remain - seg1;
		seg1 = seg1 / 100.0f;
		float seg2 = floor(remain);
		remain = remain - seg2;
		seg2 = seg2 / 100.0f;
		float seg3 = floor(remain);
		remain = remain - seg3;
		seg3 = seg3 / 100.0f;

		FragColor = vec4(seg0, seg1, seg2, seg3);
	}
	else */
	{
		FragColor = vec4(0.6f, 0.7f, 0.8f, 1.0f);
	}
}