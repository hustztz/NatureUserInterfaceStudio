#pragma once

#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>

template<class DataPtrT>
class NuiDataSharedPtrCircleBuffer
{
public:
	NuiDataSharedPtrCircleBuffer()
	{
		m_bufferDataPtr.set_capacity(100);
	}
	~NuiDataSharedPtrCircleBuffer()	{ Clear(); }

	void Clear()
	{
		boost::mutex::scoped_lock buff_lock (m_bMutex);
		while (!m_bufferDataPtr.empty())
		{
			std::shared_ptr<DataPtrT> pData = m_bufferDataPtr.front();
			m_bufferDataPtr.pop_front ();
			if(pData)
			{
				pData.reset();
			}
		}
		m_bufferDataPtr.clear();
	}

	size_t	Size()
	{
		//boost::mutex::scoped_lock buff_lock (m_bMutex);
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

	bool PushBack(std::shared_ptr<DataPtrT> pData)
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

	std::shared_ptr<DataPtrT> GetFrontPopDataPtr()
	{
		std::shared_ptr<DataPtrT> buffer(NULL);

		boost::mutex::scoped_lock buff_lock (m_bMutex);
		if( !m_bufferDataPtr.empty() )
		{
			buffer = m_bufferDataPtr.front();
			m_bufferDataPtr.pop_front ();
		}
		return buffer;
	}

	std::shared_ptr<DataPtrT> GetBackDataPtr()
	{
		std::shared_ptr<DataPtrT> buffer(NULL);

		boost::mutex::scoped_lock buff_lock (m_bMutex);
		if( !m_bufferDataPtr.empty() )
		{
			buffer = m_bufferDataPtr.back();
		}

		return buffer;
	}

	std::shared_ptr<DataPtrT> GetBackDataPtrBeforeClear()
	{
		std::shared_ptr<DataPtrT> buffer(NULL);

		boost::mutex::scoped_lock buff_lock (m_bMutex);
		if( !m_bufferDataPtr.empty() )
		{
			buffer = m_bufferDataPtr.back();
			m_bufferDataPtr.pop_back();

			while (!m_bufferDataPtr.empty())
			{
				std::shared_ptr<DataPtrT> pData = m_bufferDataPtr.front();
				m_bufferDataPtr.pop_front ();
				if(pData)
				{
					pData.reset();
				}
			}
			m_bufferDataPtr.clear();
		}

		return buffer;
	}

private:
	NuiDataSharedPtrCircleBuffer (const NuiDataSharedPtrCircleBuffer&); // Disabled copy constructor
	NuiDataSharedPtrCircleBuffer& operator = (const NuiDataSharedPtrCircleBuffer&); // Disabled assignment operator
private:
	boost::mutex										m_bMutex;
	boost::circular_buffer< std::shared_ptr<DataPtrT> >	m_bufferDataPtr;
};
