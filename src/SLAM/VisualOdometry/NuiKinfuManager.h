#pragma once

#include "Foundation\NuiThreadObject.h"
#include "NuiKinfuMainEngine.h"
#include "Frame/Buffer/NuiVisualFrameCircleBuffer.h"

class NuiKinfuManager : public NuiThreadObject
{
public:
	NuiKinfuManager();
	virtual ~NuiKinfuManager();

	virtual void	reset() override;

	bool	pushbackFrame(std::shared_ptr<NuiVisualFrame> pFrame) { return m_buffer.pushbackVisualFrame(pFrame); }
	size_t	getLagFrames() { return m_buffer.size(); }

	NuiKinfuEngine::NuiKinfuMainEngine	m_engine;
private:
	virtual bool process() override;

private:
	NuiVisualFrameCircleBuffer			m_buffer;
	bool								m_bAutoReset;
};