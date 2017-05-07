#pragma once

#include <Foundation/SgVec3T.h>
#include <Foundation/SgVec4T.h>
#include <vector>

#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>

class NuiKinfuPointCloudCache
{
public:

	void		clear() { m_vertices.clear(); m_colors.clear(); }

	int			pointSize() const { return (int)m_vertices.size(); }
	void		resizePoints(size_t size) { m_vertices.resize(size); m_colors.resize(size); }
	const SgVec3f*	getVertices() const { return m_vertices.data(); }
	const SgVec4f*	getColors() const { return m_colors.data(); }

	void		readLock() { m_mutex.lock_shared(); }
	void		readUnlock() { m_mutex.unlock_shared(); }
	void		writeLock() { m_mutex.lock(); }
	void		writeUnlock() { m_mutex.unlock(); }

private:
	std::vector<SgVec3f> m_vertices;
	std::vector<SgVec4f> m_colors;

	boost::shared_mutex		m_mutex;
};
