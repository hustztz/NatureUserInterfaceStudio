#pragma once
#include "stdafx.h"

template<typename T>
class NuiMemoryBlock
{
public:
	NuiMemoryBlock()
		: m_pBuffer(nullptr)
		, m_bufferNum(0)
	{}
	~NuiMemoryBlock()
	{
		Clear();
	}

	void			Clear()
	{
		m_bufferNum = 0;
		SafeDeleteArray(m_pBuffer);
	}
	NuiMemoryBlock (const NuiMemoryBlock& other){ DeepCopy(other); }
	NuiMemoryBlock& operator = (const NuiMemoryBlock& other) {	DeepCopy(other); return *this; }

	T*				AllocateBuffer(size_t num)
	{
		if(m_pBuffer)
		{
			if(m_bufferNum != num)
			{
				Clear();
			}
		}
		if(!m_pBuffer)
		{
			m_bufferNum = num;
			if(0 != m_bufferNum)
			{
				m_pBuffer = new T[m_bufferNum];
			}
		}
		return m_pBuffer;
	}
	T*				GetBuffer() const { return m_pBuffer; }
	size_t			GetElementBytes() const { return sizeof(T); }
	size_t			GetBufferNum() const { return m_bufferNum; }
	size_t			GetBufferSize() const { return (GetElementBytes() * GetBufferNum()); }

protected:
	void			DeepCopy (const NuiMemoryBlock& other)
	{
		if(other.m_pBuffer)
		{
			T* pBuffer = AllocateBuffer(other.m_bufferNum);
			memcpy(pBuffer, other.m_pBuffer, GetBufferSize());
		}
		else
		{
			Clear();
		}
	}

private:
	T*				m_pBuffer;
	size_t			m_bufferNum;
};
