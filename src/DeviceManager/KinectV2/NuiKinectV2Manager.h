#pragma once

#include "..\NuiRGBDDeviceManagerImpl.h"
#include "Frame\NuiImageFrame.h"
#include "Shape\NuiCompoundImage.h"

#include <Kinect.h>
#include <Kinect.VisualGestureBuilder.h>
#include <Kinect.Face.h>
#include <vector>
#include <map>

// Forwards
class NuiCompositeFrame;
class NuiRGBDDeviceBufferImpl;

class NuiKinectV2Manager : public NuiRGBDDeviceManagerImpl
{
private:
	static const int	cFPStimeInterval = 20;
	static const int	cDepthFps = 30;
	static const int	cHalfADepthFrameMs = (1000 / cDepthFps) * 2;
public:
	NuiKinectV2Manager(NuiRGBDDeviceBufferImpl* pBuffer);
	~NuiKinectV2Manager();

	virtual bool		InitializeDevice(DWORD initializeFlag) override;
	virtual bool		IsDeviceOn() const override;
	virtual bool		StartDevice() override;
	virtual void		PauseDevice() override;
	virtual void		ShutdownDevice() override;

protected:
	static DWORD WINAPI GrabThread (LPVOID pParam);
	bool				GrabFrame (NuiCompositeFrame* pCompositeFrame);
	bool				GrabDepthFrame (IMultiSourceFrame* pMultiSourceFrame);
	bool				GrabColorFrame (IMultiSourceFrame* pMultiSourceFrame);
	bool				GrabBodyIndexFrame (IMultiSourceFrame* pMultiSourceFrame);
	bool				GrabBodyFrame (IMultiSourceFrame* pMultiSourceFrame, NuiCompositeFrame* pCompositeFrame);
	bool				GrabDepthToCameraMap (NuiCompositeFrame* pCompositeFrame);
	bool				GrabDepthToColorMap (NuiCompositeFrame* pCompositeFrame);
	bool				GrabColorToDepthMap (NuiCompositeFrame* pCompositeFrame);

	bool				DetectGestures(NuiCompositeFrame* pCompositeFrame);
	bool				DetectFaces(NuiCompositeFrame* pCompositeFrame);
	bool				DetectFacialModels(NuiCompositeFrame* pCompositeFrame);

	bool				InitializeColorFrame();
	bool				InitializeDepthFrame();
	bool				InitializeBodyIndexFrame();
	bool				LoadGestureDatabase(const wchar_t* databaseFileName);
	bool				InitializeGestureSource();
	bool				InitializeFaceTracking();
	bool				InitializeFacialModel();

	bool				OnCoordinateMappingChanged(NuiCompositeFrame* pCompositeFrame);

private:
	NuiKinectV2Manager (const NuiKinectV2Manager&); // Disabled copy constructor
	NuiKinectV2Manager& operator = (const NuiKinectV2Manager&); // Disabled assignment operator

private:
	DWORD						m_eDeviceFlags;
	NuiRGBDDeviceBufferImpl*		m_pBuffer;

	IKinectSensor*				m_pNuiSensor;
	IMultiSourceFrameReader*	m_pMultiSourceFrameReader;
	ICoordinateMapper*			m_pCoordinateMapper;

	/// <summary>
	/// For depth distortion correction
	/// </summary>
	DepthSpacePoint*            m_pDepthDistortionMap;
	WAITABLE_HANDLE             m_coordinateMappingChangedEvent;

	HANDLE						m_hThNuiProcess;
	HANDLE						m_hEvNuiProcessStop;
	WAITABLE_HANDLE				m_hWaitableHandle;
	
	double						m_fFreq;

	NuiDepthFrame				m_depthFrameBuffer;
	int							m_nDepthIntervalCount;
	TIMESPAN					m_lastDepthTimeStamp;

	NuiColorFrame				m_colorFrameBuffer;
	int							m_nColorIntervalCount;
	TIMESPAN					m_lastColorTimeStamp;

	NuiBodyIndexFrame			m_bodyIndexFrameBuffer;
	NuiCompoundImage			m_compoundImage;

	// Gesture
	typedef std::vector<IGesture*> GESTURE_ARRAY;
	GESTURE_ARRAY				m_aGestureArray;
	IVisualGestureBuilderFrameSource* m_pGestureSources[BODY_COUNT];
	IVisualGestureBuilderFrameReader* m_pGestureReaders[BODY_COUNT];
	typedef std::map<UINT, UINT>	GESTURE_TYPE_INDEX_MAP;
	GESTURE_TYPE_INDEX_MAP		m_aGestureTypeMap;

	// Face Tracking
	IFaceFrameSource*			m_pFaceFrameSources[BODY_COUNT];
	IFaceFrameReader*			m_pFaceFrameReaders[BODY_COUNT];

	// Facial Model
	IHighDefinitionFaceFrameSource* m_pFacialModelSources[BODY_COUNT];
	IHighDefinitionFaceFrameReader* m_pFacialModelReaders[BODY_COUNT];
	IFaceAlignment*				m_pFacialModelAlignment[BODY_COUNT];
	IFaceModelBuilder*			m_pFacialModelBuilder[BODY_COUNT];
	bool						m_bFacialModelProduced[BODY_COUNT];
	IFaceModel*					m_pFacialModel[BODY_COUNT];
	UINT32						m_nFacialModelVertexNum;
	std::vector<float>			m_aFacialModelDeformations[BODY_COUNT];
};