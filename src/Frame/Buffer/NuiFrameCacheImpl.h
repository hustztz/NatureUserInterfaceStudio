#pragma once

#include "stdafx.h"

#include <memory>

// Forwards
class NuiCompositeFrame;

class NuiFrameCacheImpl
{
public:
	NuiFrameCacheImpl ()
	{}
	virtual ~NuiFrameCacheImpl () {}

	virtual void								clear() = 0;

	virtual bool								pushbackFrame(std::shared_ptr<NuiCompositeFrame> frame) = 0;
	virtual std::shared_ptr<NuiCompositeFrame>	getLatestFrame() = 0;
	virtual std::shared_ptr<NuiCompositeFrame>	getFrame(UINT frameId) = 0;

private:
	NuiFrameCacheImpl (const NuiFrameCacheImpl&); // Disabled copy constructor
	NuiFrameCacheImpl& operator = (const NuiFrameCacheImpl&); // Disabled assignment operator
};