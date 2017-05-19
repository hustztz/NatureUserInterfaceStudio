#pragma once

#include "NuiDataSharedPtrCircleBuffer.h"
#include "../NuiVisualFrame.h"

class NuiVisualFrameCircleBuffer
{
	static const size_t						DEVICE_BUFFER_SIZE = 200;
public:
	NuiVisualFrameCircleBuffer()
	{
		m_aFramePtrs.SetCapacity(DEVICE_BUFFER_SIZE);
	}
	~NuiVisualFrameCircleBuffer()
	{
		clear();
	}

	void clear()
	{
		m_aFramePtrs.Clear();
	}

	bool	pushbackVisualFrame(std::shared_ptr<NuiVisualFrame> pFrame)
	{
		if(!pFrame)
			return false;

		if( !m_aFramePtrs.PushBack(pFrame) )
		{
			// Memory is full!
			printf_s( "The circle buffer is full!\n" );
			pFrame.reset();
			return false;
		}
		return true;
	}

	std::shared_ptr<NuiVisualFrame>	popVisualFrame()
	{
		return m_aFramePtrs.GetFrontPopDataPtr();
	}

	size_t	size()
	{
		return m_aFramePtrs.Size();
	}

private:
	NuiDataSharedPtrCircleBuffer<NuiVisualFrame>	m_aFramePtrs;
};