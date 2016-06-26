#pragma once

#include "stdafx.h"

class NuiFusionMesh
{
public:
	NuiFusionMesh()
		: m_pVertices(NULL)
		, m_vertexCount(0)
		, m_pNormals(NULL)
		, m_normalCount(0)
		, m_pTriangleIndices(NULL)
		, m_triangleIndexCount(0)
		, m_pColors(NULL)
		, m_colorCount(0)
	{};
	~NuiFusionMesh() { clear(); }

	NuiFusionMesh (const NuiFusionMesh& other)
	{
		deepCopy(other);
	}
	NuiFusionMesh& operator = (const NuiFusionMesh& other)
	{
		deepCopy(other);
		return *this;
	}

	void deepCopy (const NuiFusionMesh& other)
	{
		Vector3*	pVertices = allocateVertices(other.m_vertexCount);
		if(pVertices)
		{
			memcpy(pVertices, other.m_pVertices, (sizeof(Vector3) * m_vertexCount));
		}

		Vector3*	pNormals = allocateNormals(other.m_normalCount);
		if(pNormals)
		{
			memcpy(pNormals, other.m_pNormals, (sizeof(Vector3) * m_normalCount));
		}

		int*	pTriangleIndices = allocateTriangleIndices(other.m_triangleIndexCount);
		if(pTriangleIndices)
		{
			memcpy(pTriangleIndices, other.m_pTriangleIndices, (sizeof(int) * m_triangleIndexCount));
		}

		int*	pColors = allocateColors(other.m_colorCount);
		if(pColors)
		{
			memcpy(pColors, other.m_pColors, (sizeof(int) * m_colorCount));
		}
	}

	void clear()
	{
		if(m_pVertices)
		{
			_freea(m_pVertices);
			m_pVertices = NULL;
		}
		m_vertexCount = 0;
		if(m_pNormals)
		{
			_freea(m_pNormals);
			m_pNormals = NULL;
		}
		m_normalCount = 0;
		if(m_pTriangleIndices)
		{
			_freea(m_pTriangleIndices);
			m_pTriangleIndices = NULL;
		}
		m_triangleIndexCount = 0;
		if(m_pColors)
		{
			_freea(m_pColors);
			m_pColors = NULL;
		}
		m_colorCount = 0;
	}

	Vector3* allocateVertices(unsigned int count)
	{
		if(count != m_vertexCount)
		{
			if(m_pVertices)
				_freea(m_pVertices);
			m_pVertices = NULL;
		}
		m_vertexCount = count;
		if(!m_pVertices && m_vertexCount != 0)
			m_pVertices = reinterpret_cast<Vector3*>(_malloca(sizeof(Vector3) * m_vertexCount));
		return m_pVertices;
	}
	Vector3* getVertices() const { return m_pVertices; }
	unsigned int getVertexCount() const { return m_vertexCount; }

	Vector3* allocateNormals(unsigned int count)
	{
		if(count != m_normalCount)
		{
			if(m_pNormals)
				_freea(m_pNormals);
			m_pNormals = NULL;
		}
		m_normalCount = count;
		if(!m_pNormals && m_normalCount != 0)
			m_pNormals = reinterpret_cast<Vector3*>(_malloca(sizeof(Vector3) * m_normalCount));
		return m_pNormals;
	}
	Vector3* getNormals() const { return m_pNormals; }
	unsigned int getNormalCount() const { return m_normalCount; }

	int* allocateTriangleIndices(unsigned int count)
	{
		if(count != m_triangleIndexCount)
		{
			if(m_pTriangleIndices)
				_freea(m_pTriangleIndices);
			m_pTriangleIndices = NULL;
		}
		m_triangleIndexCount = count;
		if(!m_pTriangleIndices && m_triangleIndexCount != 0)
			m_pTriangleIndices = reinterpret_cast<int*>(_malloca(sizeof(int) * m_triangleIndexCount));
		return m_pTriangleIndices;
	}
	int* getTriangleIndices() const { return m_pTriangleIndices; }
	unsigned int getTriangleIndexCount() const { return m_triangleIndexCount; }

	int* allocateColors(unsigned int count)
	{
		if(count != m_colorCount)
		{
			if(m_pColors)
				_freea(m_pColors);
			m_pColors = NULL;
		}
		m_colorCount = count;
		if(!m_pColors && m_colorCount != 0)
			m_pColors = reinterpret_cast<int*>(_malloca(sizeof(int) * m_colorCount));
		return m_pColors;
	}
	int* getColors() const { return m_pColors; }
	unsigned int getColorCount() const { return m_colorCount; }

private:
	Vector3*					m_pVertices;
	unsigned int				m_vertexCount;
	Vector3*					m_pNormals;
	unsigned int				m_normalCount;
	int*						m_pTriangleIndices;
	unsigned int				m_triangleIndexCount;
	int*						m_pColors;
	unsigned int				m_colorCount;
};
