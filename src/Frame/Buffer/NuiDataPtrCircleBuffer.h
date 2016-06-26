#pragma once

#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>

template<class DataPtrT>
class NuiDataPtrCircleBuffer
{
public:
	NuiDataPtrCircleBuffer()
	{
		m_bufferDataPtr.set_capacity(100);
	}
	~NuiDataPtrCircleBuffer()	{ Clear(); }

	void Clear()
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		while (!m_bufferDataPtr.empty())
		{
			DataPtrT* pData = m_bufferDataPtr.front();
			m_bufferDataPtr.pop_front ();
			if(pData)
			{
				delete pData;
				pData = NULL;
			}
		}
		m_bufferDataPtr.clear();
	}

	size_t	Size()
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		return m_bufferDataPtr.size();
	}

	bool IsFull ()
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		return (m_bufferDataPtr.full ());
	}

	bool IsEmpty ()
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		return (m_bufferDataPtr.empty ());
	}

	void SetCapacity(size_t capacity)
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		m_bufferDataPtr.set_capacity(capacity);
	}

	bool PushBack(DataPtrT* pData)
	{
		if(!pData)
			return false;

		boost::mutex::scoped_lock buff_lock (m_bMutex);
		if(m_bufferDataPtr.full())
		{
			std::cout << "Circle buffer is full!" << std::endl;
			return false;
		}
		m_bufferDataPtr.push_back(pData);
		return true;
	}

	DataPtrT* GetFrontPopDataPtr()
	{
		DataPtrT* buffer = NULL;

		boost::mutex::scoped_lock buff_lock (m_bMutex);
		if( !m_bufferDataPtr.empty() )
		{
			buffer = m_bufferDataPtr.front();
			m_bufferDataPtr.pop_front ();
		}
		return buffer;
	}

	DataPtrT* GetBackDataPtr()
	{
		DataPtrT* buffer = NULL;

		boost::mutex::scoped_lock buff_lock (m_bMutex);
		if( !m_bufferDataPtr.empty() )
		{
			buffer = m_bufferDataPtr.back();
		}

		return buffer;
	}

	DataPtrT* GetBackDataPtrBeforeClear()
	{
		DataPtrT* buffer = NULL;

		boost::mutex::scoped_lock buff_lock (m_bMutex);
		if( !m_bufferDataPtr.empty() )
		{
			buffer = m_bufferDataPtr.back();
			m_bufferDataPtr.pop_back();

			while (!m_bufferDataPtr.empty())
			{
				DataPtrT* pData = m_bufferDataPtr.front();
				m_bufferDataPtr.pop_front ();
				if(pData)
				{
					delete pData;
					pData = NULL;
				}
			}
			m_bufferDataPtr.clear();
		}

		return buffer;
	}

private:
	NuiDataPtrCircleBuffer (const NuiDataPtrCircleBuffer&); // Disabled copy constructor
	NuiDataPtrCircleBuffer& operator = (const NuiDataPtrCircleBuffer&); // Disabled assignment operator
private:
	boost::mutex						m_bMutex;
	boost::circular_buffer<DataPtrT*>	m_bufferDataPtr;
};
