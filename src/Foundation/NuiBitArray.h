#pragma once

#include "stdafx.h"

template <class T>
class NuiBitArray
{
	public:

		NuiBitArray()
		{
			m_data = NULL;
			m_nBits = 0;
		}

		NuiBitArray(unsigned int nBits)
		{
			m_nBits = nBits;

			const unsigned int nInts = (m_nBits+nBitsInT-1)/nBitsInT;
			m_data = new T[nInts];
			for(unsigned int i = 0; i<nInts; i++)
			{
				m_data[i] = T();
			}
		}
		
		bool isBitSet(unsigned int index) const
		{
			if(index < m_nBits)
			{
				return ((m_data[index/nBitsInT] & (0x1 << (index%nBitsInT))) != 0x0);
			}
			else
			{
				std::cout << "Out of Bounds" << std::endl;
				while(1);
			}
		}

		void setBit(unsigned int index)
		{
			if(index < m_nBits)
			{
				m_data[index/nBitsInT] |= (0x1 << (index%nBitsInT));
			}
			else
			{
				std::cout << "Out of Bounds" << std::endl;
				while(1);
			}
		}
		
		void resetBit(unsigned int index)
		{
			if(index < m_nBits)
			{
				m_data[index/nBitsInT] &= ~(0x1 << (index%nBitsInT));
			}
			else
			{
				std::cout << "Out of Bounds" << std::endl;
				while(1);
			}
		}

		void reset()
		{
			const unsigned int nInts = (m_nBits+nBitsInT-1)/nBitsInT;
			for(unsigned int i = 0; i<nInts; i++)
			{
				m_data[i] = T();
			}
		}

		const T* getRawData() const
		{
			return m_data;
		}

		unsigned int getNBits() const
		{
			return m_nBits;
		}

		unsigned int getByteWidth() const
		{
			return sizeof(T)*((m_nBits+nBitsInT-1)/nBitsInT);
		}
		
		NuiBitArray(const NuiBitArray<T>& other)
        {
			*this = other;
        }
        
        void operator=(const NuiBitArray<T>& other)
        {
			if(this != &other)
			{
				SafeDelete(m_data);
				m_nBits = other.getNBits();
				m_data = new T[other.getByteWidth()/sizeof(T)];
				memcpy(&m_data[0], &(other.getRawData()[0]), other.getByteWidth());
			}
        }
		
		~NuiBitArray()
		{
			SafeDelete(m_data);
		}

	private:

		static const unsigned int nBitsInT = 8*sizeof(T);

		unsigned int m_nBits;
		T* m_data;
};
