#pragma once

#include "vector"
#include <boost/thread/mutex.hpp>

template<class DataPtrT>
class NuiDataPtrCache
{
public:
	NuiDataPtrCache() : m_capacity(200){};
	~NuiDataPtrCache()	{ Clear(); }

	void Clear()
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		for (std::vector<DataPtrT*>::iterator iter=m_arrDataPtr.begin();iter!=m_arrDataPtr.end();++iter)
		{
			DataPtrT* pData = *iter;
			if(pData)
			{
				delete pData;
				pData = NULL;
			}
		}
		m_arrDataPtr.clear();
	}

	size_t	Size()
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		return m_arrDataPtr.size();
	}

	void SetCapacity(size_t capacity) { m_capacity = capacity; }

	bool PushBack(DataPtrT* pData)
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		if(pData && m_arrDataPtr.size() < m_capacity)
		{
			m_arrDataPtr.push_back(pData);
			return true;
		}
		return false;
	}

	DataPtrT* GetDataPtr(int frameId)
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		if(frameId >= 0 && frameId < m_arrDataPtr.size() )
			return m_arrDataPtr.at(frameId);

		return NULL;
	}

	DataPtrT* GetFrontDataPtr()
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		if(!m_arrDataPtr.empty() )
			return m_arrDataPtr.front();

		return NULL;
	}

	DataPtrT* GetBackDataPtr()
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		if(!m_arrDataPtr.empty() )
			return m_arrDataPtr.back();

		return NULL;
	}

private:
	NuiDataPtrCache (const NuiDataPtrCache&); // Disabled copy constructor
	NuiDataPtrCache& operator = (const NuiDataPtrCache&); // Disabled assignment operator
private:
	boost::mutex				m_bMutex;
	std::vector<DataPtrT*>		m_arrDataPtr;
	size_t						m_capacity;
};
