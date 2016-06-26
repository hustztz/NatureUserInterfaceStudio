#pragma once

#include <Foundation/SgVec2T.h>
#include <Foundation/SgVec3T.h>
#include <vector>

class NuiMeshShape
{
public:
	NuiMeshShape(){}
	~NuiMeshShape(){}

	void appendPoint(const SgVec3f& point, const SgVec2f& uv, const SgVec3f& color)
	{
		m_points.push_back(point);
		m_UVs.push_back(uv);
		m_colors.push_back(color);
	}

	void appendPoint(const SgVec3f& point)
	{
		m_points.push_back(point);
	}

	void appendTriangleIndex(int index)
	{
		m_triangle_indices.push_back(index);
	}

	size_t pointsNum() const { return m_points.size(); }
	const SgVec3f& getPoint(unsigned int id) const { return m_points.at(id); }
	const SgVec2f& getUV(unsigned int id) const
	{
		if(id >= m_UVs.size())
			return sInvalidUV;
		return m_UVs.at(id);
	}
	const SgVec3f& getColor(unsigned int id) const
	{
		if(id >= m_colors.size())
			return sInvalidColor;
		return m_colors.at(id);
	}

	size_t trianglesNum() const { return (m_triangle_indices.size() / 3); }
	int triangleIndex(unsigned int id) const
	{
		if(id >= m_triangle_indices.size())
			return -1;
		return m_triangle_indices.at(id);
	}

private:
	std::vector<SgVec3f> m_points;
	std::vector<int> m_triangle_indices;
	std::vector<SgVec2f> m_UVs;
	std::vector<SgVec3f> m_colors;

	SgVec2f sInvalidUV;
	SgVec3f sInvalidColor;
};
