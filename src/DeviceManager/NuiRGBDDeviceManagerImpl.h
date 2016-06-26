#pragma once

class NuiRGBDDeviceManagerImpl
{
public:
	enum eRGBDDeviceFlags
	{
		EDevice_None = 0,
		EDevice_Camera_Intrisics = (0x1 << 0),
		EDevice_Depth_On = (0x1 << 1),
		EDevice_Color_On = (0x1 << 2),
		EDevice_ColorMap_On = (0x1 << 3),
		EDevice_CameraMap_On = (0x1 << 4),
		EDevice_DepthMap_On = (0x1 << 5),
		EDevice_PlayIndex_On = (0x1 << 6),
		EDevice_Skeleton_On = (0x1 << 7),
		EDevice_Face_On = (0x1 << 8),
		EDevice_FacialModel_On = (0x1 << 9),
		EDevice_Fusion_On = (0x1 << 10),
		EDevice_Gesture_On = (0x1 << 11),

		Expand_LAST_BIT = (0x1 << 12),

	};

	NuiRGBDDeviceManagerImpl(){};
	virtual ~NuiRGBDDeviceManagerImpl(){};

	virtual bool			InitializeDevice(unsigned long initializeFlag) = 0;
	virtual bool			IsDeviceOn() const = 0;
	virtual bool			StartDevice () = 0;
	virtual void			PauseDevice () = 0;
	virtual void			ShutdownDevice () = 0;

	virtual bool			UpdateNearMode(bool bNearMode) { return false; }
	virtual void			UpdateElevationAngle(long tiltAngle) {};
	virtual void			UpdateSkeletonSmoothParams(float smoothing, float correction, float prediction, float jitterRadius, float maxDeviationRadius) {};
};