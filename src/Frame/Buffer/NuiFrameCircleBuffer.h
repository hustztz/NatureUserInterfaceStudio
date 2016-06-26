#pragma once

#include "NuiDataSharedPtrCircleBuffer.h"
#include "../NuiCompositeFrame.h"

class NuiFrameCircleBuffer
{
	static const size_t						DEVICE_BUFFER_SIZE = 200;
public:
	NuiFrameCircleBuffer ()
	{
		m_aFramePtrs.SetCapacity(DEVICE_BUFFER_SIZE);
	}
	~NuiFrameCircleBuffer ()
	{
		clear();
	}

	void clear()
	{
		m_aFramePtrs.Clear();
	}

	bool	pushbackCompositeFrame(std::shared_ptr<NuiCompositeFrame> pFrame)
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

	std::shared_ptr<NuiCompositeFrame>	popCompositeFrame()
	{
		return m_aFramePtrs.GetFrontPopDataPtr();
	}

	size_t	size()
	{
		return m_aFramePtrs.Size();
	}

private:
	NuiDataSharedPtrCircleBuffer<NuiCompositeFrame>	m_aFramePtrs;
};