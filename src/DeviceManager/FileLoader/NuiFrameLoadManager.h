#pragma once
#include "stdafx.h"

#include "..\NuiRGBDDeviceManagerImpl.h"

#include "NuiFrameLoader.h"

class NuiFrameLoadManager : public NuiRGBDDeviceManagerImpl
{
public:
	NuiFrameLoadManager(NuiRGBDDeviceBufferImpl* pBuffer, const std::string&	fileName);
	virtual ~NuiFrameLoadManager();

	virtual bool		InitializeDevice(DWORD initializeFlag) override;
	virtual bool		IsDeviceOn() const override;
	virtual bool		StartDevice() override;
	virtual void		PauseDevice() override;
	virtual void		ShutdownDevice() override;

private:
	NuiFrameLoader			m_loader;
};