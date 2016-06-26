#pragma once
#include "stdafx.h"

#include "Foundation\NuiThreadObject.h"

#include <string>

// Forwards
class NuiRGBDDeviceBufferImpl;

class NuiFrameLoader : public NuiThreadObject
{
public:
	NuiFrameLoader();
	virtual ~NuiFrameLoader();

	void reset() override;

	void updateFileName(const std::string&	fileName) { m_fileName = fileName; }
	void updateBuffer(NuiRGBDDeviceBufferImpl* pBuffer) { m_pBuffer = pBuffer; }
protected:
	virtual bool process() override;

private:
	NuiRGBDDeviceBufferImpl*	m_pBuffer;
	std::string				m_fileName;
	UINT					m_frameId;
};