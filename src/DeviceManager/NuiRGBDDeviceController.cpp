#include "NuiRGBDDeviceController.h"

#include "DeviceManager/KinectV2/NuiKinectV2Manager.h"
#include "DeviceManager/FileLoader/NuiFrameLoadManager.h"

NuiRGBDDeviceController::NuiRGBDDeviceController()
	: m_pDevice(NULL)
{
}

NuiRGBDDeviceController::~NuiRGBDDeviceController()
{
	SafeDelete(m_pDevice);
}

bool NuiRGBDDeviceController::initializeDevice(DWORD deviceMode)
{
	DWORD deviceFlag = 0;
	switch (deviceMode)
	{
	case EDeviceMode_VertexColorCamera:
		deviceFlag = NuiRGBDDeviceManagerImpl::EDevice_Depth_On |
			NuiRGBDDeviceManagerImpl::EDevice_CameraMap_On |
			NuiRGBDDeviceManagerImpl::EDevice_Color_On |
			NuiRGBDDeviceManagerImpl::EDevice_ColorMap_On |
			NuiRGBDDeviceManagerImpl::EDevice_Camera_Intrisics;
		break;
	case EDeviceMode_Color:
		deviceFlag = NuiRGBDDeviceManagerImpl::EDevice_Color_On;
		break;
	case EDeviceMode_Vertex:
		deviceFlag = NuiRGBDDeviceManagerImpl::EDevice_Depth_On |
			NuiRGBDDeviceManagerImpl::EDevice_CameraMap_On |
			NuiRGBDDeviceManagerImpl::EDevice_PlayIndex_On;
		break;
	case EDeviceMode_Skeleton:
		deviceFlag = NuiRGBDDeviceManagerImpl::EDevice_Skeleton_On;
		break;
	case EDeviceMode_DepthColor:
		deviceFlag = NuiRGBDDeviceManagerImpl::EDevice_Depth_On |
			NuiRGBDDeviceManagerImpl::EDevice_Color_On |
			NuiRGBDDeviceManagerImpl::EDevice_ColorMap_On;
		break;
	case EDeviceMode_VertexColorPlayer:
		deviceFlag = NuiRGBDDeviceManagerImpl::EDevice_Depth_On |
			NuiRGBDDeviceManagerImpl::EDevice_CameraMap_On |
			NuiRGBDDeviceManagerImpl::EDevice_Color_On |
			NuiRGBDDeviceManagerImpl::EDevice_ColorMap_On |
			NuiRGBDDeviceManagerImpl::EDevice_PlayIndex_On;
		break;
	case EDeviceMode_VertexColorSkeleton:
		deviceFlag = NuiRGBDDeviceManagerImpl::EDevice_Depth_On |
			NuiRGBDDeviceManagerImpl::EDevice_CameraMap_On |
			NuiRGBDDeviceManagerImpl::EDevice_Color_On |
			NuiRGBDDeviceManagerImpl::EDevice_ColorMap_On |
			NuiRGBDDeviceManagerImpl::EDevice_PlayIndex_On |
			NuiRGBDDeviceManagerImpl::EDevice_Skeleton_On;
		break;
	case EDeviceMode_VertexColorSkeletonFace:
		deviceFlag = NuiRGBDDeviceManagerImpl::EDevice_Depth_On |
			NuiRGBDDeviceManagerImpl::EDevice_CameraMap_On |
			NuiRGBDDeviceManagerImpl::EDevice_Color_On |
			NuiRGBDDeviceManagerImpl::EDevice_ColorMap_On |
			NuiRGBDDeviceManagerImpl::EDevice_PlayIndex_On |
			NuiRGBDDeviceManagerImpl::EDevice_Skeleton_On |
			NuiRGBDDeviceManagerImpl::EDevice_FacialModel_On; //EDevice_Face_On
		break;
	case EDeviceMode_VertexColorSkeletonGesture:
		deviceFlag = NuiRGBDDeviceManagerImpl::EDevice_Depth_On |
			NuiRGBDDeviceManagerImpl::EDevice_CameraMap_On |
			NuiRGBDDeviceManagerImpl::EDevice_Color_On |
			NuiRGBDDeviceManagerImpl::EDevice_ColorMap_On |
			NuiRGBDDeviceManagerImpl::EDevice_PlayIndex_On |
			NuiRGBDDeviceManagerImpl::EDevice_Skeleton_On |
			NuiRGBDDeviceManagerImpl::EDevice_Gesture_On;
		break;
	case EDeviceMode_Fusion:
		deviceFlag = NuiRGBDDeviceManagerImpl::EDevice_Depth_On |
			NuiRGBDDeviceManagerImpl::EDevice_Color_On |
			NuiRGBDDeviceManagerImpl::EDevice_ColorMap_On |
			NuiRGBDDeviceManagerImpl::EDevice_Fusion_On;
		break;
	default:
		assert(0);
		return false;
	}

	SafeDelete(m_pDevice);

	m_pDevice = new NuiKinectV2Manager(&m_buffer);
	assert(m_pDevice);

	if( !m_pDevice->InitializeDevice(deviceFlag) )
	{
		std::cerr << "Error : Failed to initialize the device." << std::endl;
		SafeDelete(m_pDevice);
		return false;
	}
	return true;
}

bool NuiRGBDDeviceController::initializeFileLoader(const std::string&	fileName)
{
	DWORD deviceFlag = 0;
	
	SafeDelete(m_pDevice);

	m_pDevice = new NuiFrameLoadManager(&m_buffer, fileName);
	assert(m_pDevice);

	if( !m_pDevice->InitializeDevice(deviceFlag) )
	{
		std::cerr << "Error : Failed to initialize the device." << std::endl;
		SafeDelete(m_pDevice);
		return false;
	}
	return true;
}

bool NuiRGBDDeviceController::startDevice()
{
	if (!m_pDevice)
		return false;

	if (!m_pDevice->StartDevice())
	{
		std::cerr << "Error : Failed to start the device." << std::endl;
		SafeDelete(m_pDevice);
		return false;
	}
	return true;
}

void NuiRGBDDeviceController::pauseDevice()
{
	if(m_pDevice)
		m_pDevice->PauseDevice();
}

void NuiRGBDDeviceController::stopDevice()
{
	if(m_pDevice)
		m_pDevice->ShutdownDevice();
}

bool NuiRGBDDeviceController::UpdateNearMode(bool bNearMode)
{
	 return m_pDevice ?	m_pDevice->UpdateNearMode(bNearMode) : false;
}

void NuiRGBDDeviceController::UpdateElevationAngle(long tiltAngle)
{
	if(m_pDevice)
		m_pDevice->UpdateElevationAngle(tiltAngle);
}

std::shared_ptr<NuiCompositeFrame>	NuiRGBDDeviceController::popFrame()
{
	if(m_buffer.size() > 1)
		return m_buffer.popFrame();
	return std::shared_ptr<NuiCompositeFrame>(NULL);
}

