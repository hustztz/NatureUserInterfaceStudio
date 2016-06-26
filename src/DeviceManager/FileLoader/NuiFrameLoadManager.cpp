#include "NuiFrameLoadManager.h"

NuiFrameLoadManager::NuiFrameLoadManager(NuiRGBDDeviceBufferImpl* pBuffer, const std::string&	fileName)
{
	m_loader.updateFileName(fileName);
	m_loader.updateBuffer(pBuffer);
}

NuiFrameLoadManager::~NuiFrameLoadManager()
{
	ShutdownDevice();
}

bool	NuiFrameLoadManager::InitializeDevice(DWORD initializeFlag)
{
	m_loader.reset();
	return true;
}

bool	NuiFrameLoadManager::IsDeviceOn() const
{
	return m_loader.isThreadOn();
}

bool	NuiFrameLoadManager::StartDevice()
{
	m_loader.startThread();
	return true;
}

void	NuiFrameLoadManager::PauseDevice()
{
	m_loader.pauseThread();
}

void	NuiFrameLoadManager::ShutdownDevice()
{
	m_loader.stopThread();
}