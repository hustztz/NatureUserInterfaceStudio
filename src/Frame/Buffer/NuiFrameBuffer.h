#pragma once

#include "NuiFrameCacheImpl.h"

class NuiFrameBuffer : public NuiFrameCacheImpl
{
public:
	NuiFrameBuffer();
	virtual ~NuiFrameBuffer();

	virtual void								clear() override;

	virtual bool								pushbackFrame(std::shared_ptr<NuiCompositeFrame> pFrame) override;
	virtual std::shared_ptr<NuiCompositeFrame>	getLatestFrame() override;
	virtual std::shared_ptr<NuiCompositeFrame>	getFrame(UINT frameId) override;

private:
	std::shared_ptr<NuiCompositeFrame>			m_pFrame;
};