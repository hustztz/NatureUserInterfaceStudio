#pragma once

#include "Foundation\NuiThreadObject.h"

#include "Buffer/NuiVisualFrameCircleBuffer.h"

class NuiVisualFrameSaveManager : public NuiThreadObject
{
public:
	NuiVisualFrameSaveManager(const std::string&	fileName);
	virtual ~NuiVisualFrameSaveManager();

	virtual void	reset() override;
	bool	pushbackFrame(std::shared_ptr<NuiVisualFrame> pFrame) { return m_buffer.pushbackVisualFrame(pFrame); }
	size_t	getLagFrames() { return m_buffer.size(); }

protected:
	virtual bool process() override;

private:
	NuiVisualFrameCircleBuffer	m_buffer;
	std::string				m_fileName;
	UINT					m_bCompressed;
};