#pragma once

#include "vector"
#include <boost/thread/mutex.hpp>

template<class DataPtrT>
class NuiDataSharedPtrCache
{
public:
	NuiDataSharedPtrCache() : m_capacity(200){};
	~NuiDataSharedPtrCache()	{ Clear(); }

	void Clear()
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		for (std::vector< std::shared_ptr<DataPtrT> >::iterator iter=m_arrDataPtr.begin();iter!=m_arrDataPtr.end();++iter)
		{
			std::shared_ptr<DataPtrT> pData = *iter;
			pData.reset();
		}
		m_arrDataPtr.clear();
	}

	size_t	Size()
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		return m_arrDataPtr.size();
	}

	void SetCapacity(size_t capacity) { m_capacity = capacity; }

	bool PushBack(std::shared_ptr<DataPtrT> pData)
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		if(pData && m_arrDataPtr.size() < m_capacity)
		{
			m_arrDataPtr.push_back(pData);
			return true;
		}
		return false;
	}

	std::shared_ptr<DataPtrT> GetDataPtr(int frameId)
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		if(frameId >= 0 && frameId < m_arrDataPtr.size() )
			return m_arrDataPtr.at(frameId);

		return NULL;
	}

	std::shared_ptr<DataPtrT> GetFrontDataPtr()
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		if(!m_arrDataPtr.empty() )
			return m_arrDataPtr.front();

		return NULL;
	}

	std::shared_ptr<DataPtrT> GetBackDataPtr()
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		if(!m_arrDataPtr.empty() )
			return m_arrDataPtr.back();

		return NULL;
	}

private:
	NuiDataSharedPtrCache (const NuiDataSharedPtrCache&); // Disabled copy constructor
	NuiDataSharedPtrCache& operator = (const NuiDataSharedPtrCache&); // Disabled assignment operator
private:
	boost::mutex				m_bMutex;
	std::vector< std::shared_ptr<DataPtrT> >		m_arrDataPtr;
	size_t						m_capacity;
};
