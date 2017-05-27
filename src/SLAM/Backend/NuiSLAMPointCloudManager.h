#pragma once

#include "Foundation\NuiThreadObject.h"
#include "NuiSLAMPointCloud.h"
#include "Shape/NuiPolygonMesh.h"

#include <boost/thread/mutex.hpp>

class NuiKinfuVertexCache;

namespace NuiSLAMEngine
{
	class NuiSLAMPointCloudManager : public NuiThreadObject
	{
	public:
		NuiSLAMPointCloudManager();
		virtual ~NuiSLAMPointCloudManager();

		virtual void	reset() override;

		void	setVertexCache(NuiKinfuVertexCache* pCache) { m_pVertexCache = pCache; }
		void	setFilterLeafSize(float size) { m_filterLeafSize = size; }

	private:
		virtual bool process() override;

	private:
		NuiKinfuVertexCache*			m_pVertexCache;
		NuiSLAMPointCloud				m_pointCloud;
		NuiPolygonMesh					m_mesh;
		float							m_filterLeafSize;
	};
}