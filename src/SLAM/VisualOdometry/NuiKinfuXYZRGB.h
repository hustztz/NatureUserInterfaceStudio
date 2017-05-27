#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>

class NuiKinfuVertexCache;

class NuiKinfuXYZRGB
{
public:
	NuiKinfuXYZRGB();
	~NuiKinfuXYZRGB();

	void		clear() { m_pointCloud.clear(); }

	int			pointSize() const { return (int)m_pointCloud.size(); }
	void		resizePoints(size_t size) { m_pointCloud.resize(size); }
	bool		incrementPoints(NuiKinfuVertexCache* pVertexCache);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr	getPtr() const { return m_pointCloud.makeShared(); }

	void		readLock() { m_mutex.lock_shared(); }
	void		readUnlock() { m_mutex.unlock_shared(); }
	void		writeLock() { m_mutex.lock(); }
	void		writeUnlock() { m_mutex.unlock(); }

private:
	pcl::PointCloud<pcl::PointXYZRGB>	m_pointCloud;

	boost::shared_mutex		m_mutex;
};
