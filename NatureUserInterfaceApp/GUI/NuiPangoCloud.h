#pragma once

#include <pangolin/pangolin.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>

struct NuiPointXYZRGB
{
	float x;
	float y;
	float z;

	unsigned char r;
	unsigned char g;
	unsigned char b;
	unsigned char a;
};

class NuiPangoCloud
{
public:
	NuiPangoCloud(NuiPointXYZRGB* cloud, int numPoints)
		: m_numPoints(numPoints)
		, m_offset(3)
	{
		glGenBuffers(1, &m_vbo);
		glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
		glBufferData(GL_ARRAY_BUFFER, m_numPoints * sizeof(NuiPointXYZRGB), cloud, GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	virtual ~NuiPangoCloud()
	{
		glDeleteBuffers(1, &m_vbo);
	}

	void drawPoints()
	{
		const int stride = sizeof(NuiPointXYZRGB);
		glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
		glVertexPointer(3, GL_FLOAT, stride, 0);
		glColorPointer(3, GL_UNSIGNED_BYTE, stride, (void *)(sizeof(float) * m_offset));

		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_COLOR_ARRAY);

		glPointSize(2); 
		glDrawArrays(GL_POINTS, 0, m_numPoints);

		glDisableClientState(GL_COLOR_ARRAY);
		glDisableClientState(GL_VERTEX_ARRAY);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	const int m_numPoints;

private:
	const int m_offset;
	GLuint m_vbo;

};