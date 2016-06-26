#pragma once

#include "stdafx.h"

// Forwards
class NuiCompositeFrame;

class NuiRGBDDeviceBufferImpl
{
public:
	NuiRGBDDeviceBufferImpl ()
	{}
	virtual ~NuiRGBDDeviceBufferImpl () {}

	virtual void									clear() = 0;

	virtual std::shared_ptr<NuiCompositeFrame>		allocateFrame() = 0;
	virtual std::shared_ptr<NuiCompositeFrame>		popFrame() = 0;
	virtual size_t									size() = 0;

private:
	NuiRGBDDeviceBufferImpl (const NuiRGBDDeviceBufferImpl&); // Disabled copy constructor
	NuiRGBDDeviceBufferImpl& operator = (const NuiRGBDDeviceBufferImpl&); // Disabled assignment operator
};