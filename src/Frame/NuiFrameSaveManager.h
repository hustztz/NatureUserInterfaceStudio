#pragma once

#include "Foundation\NuiThreadObject.h"

#include "Frame/Buffer/NuiFrameCircleBuffer.h"

class NuiFrameSaveManager : public NuiThreadObject
{
public:
	NuiFrameSaveManager(const std::string&	fileName);
	virtual ~NuiFrameSaveManager();

	virtual void	reset() override;
	bool	pushbackFrame(std::shared_ptr<NuiCompositeFrame> pFrame) { return m_buffer.pushbackCompositeFrame(pFrame); }
	size_t	getLagFrames() { return m_buffer.size(); }

protected:
	virtual bool process() override;

private:
	NuiFrameCircleBuffer	m_buffer;
	std::string				m_fileName;
	UINT					m_bCompressed;
	UINT					m_frameId;
};