#pragma once

#include "Frame/Buffer/NuiDataSharedPtrCircleBuffer.h"
#include "NuiRGBDDeviceBufferImpl.h"
#include "Frame/NuiCompositeFrame.h"

class NuiRGBDDeviceCircleBuffer : public NuiRGBDDeviceBufferImpl
{
	static const size_t						DEVICE_BUFFER_SIZE = 50;
public:
	NuiRGBDDeviceCircleBuffer ()
		: NuiRGBDDeviceBufferImpl()
	{
		m_aFramePtrs.SetCapacity(DEVICE_BUFFER_SIZE);
	}
	virtual ~NuiRGBDDeviceCircleBuffer ()
	{
		clear();
	}

	virtual void clear() override
	{
		m_aFramePtrs.Clear();
	}

	virtual std::shared_ptr<NuiCompositeFrame>	allocateFrame() override
	{
		std::shared_ptr<NuiCompositeFrame> pFrame = std::make_shared<NuiCompositeFrame>();
		assert(pFrame);
		if( !m_aFramePtrs.PushBack(pFrame) )
		{
			// Memory is full!
			printf_s( "The device buffer is full!\n" );
			pFrame.reset();
			return NULL;
		}
		return pFrame;
	}

	virtual std::shared_ptr<NuiCompositeFrame>	popFrame() override
	{
		return m_aFramePtrs.GetFrontPopDataPtr();
	}

	virtual size_t									size() override
	{
		return m_aFramePtrs.Size();
	}

private:
	NuiDataSharedPtrCircleBuffer<NuiCompositeFrame>	m_aFramePtrs;
};