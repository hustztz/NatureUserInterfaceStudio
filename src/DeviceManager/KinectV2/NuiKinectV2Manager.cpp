#include "stdafx.h"
#include "NuiKinectV2Manager.h"
#include "../NuiRGBDDeviceBufferImpl.h"
#include "Frame/NuiCompositeFrame.h"

// Standard Library
#include <omp.h>
#include <iostream>
#include <assert.h>
#include <vector>

NuiKinectV2Manager::NuiKinectV2Manager(NuiRGBDDeviceBufferImpl* pBuffer)
	: m_eDeviceFlags(EDevice_None)
	, m_pBuffer(pBuffer)
	, m_pNuiSensor(nullptr)
	, m_pMultiSourceFrameReader(nullptr)
	, m_pCoordinateMapper(nullptr)
	, m_hThNuiProcess(NULL)
	, m_hEvNuiProcessStop(NULL)
	, m_hWaitableHandle(NULL)
	, m_coordinateMappingChangedEvent(NULL)
	, m_nColorIntervalCount(cFPStimeInterval)
	, m_lastColorTimeStamp(0)
	, m_nDepthIntervalCount(cFPStimeInterval)
	, m_lastDepthTimeStamp(0)
	, m_pDepthDistortionMap(nullptr)
	, m_nFacialModelVertexNum(0)
{
	for( UINT count = 0; count < BODY_COUNT; count++ ){
		m_pGestureSources[count] = nullptr;
		m_pGestureReaders[count] = nullptr;
		m_pFaceFrameSources[count] = nullptr;
		m_pFaceFrameReaders[count] = nullptr;
		m_pFacialModelSources[count] = nullptr;
		m_pFacialModelReaders[count] = nullptr;
		m_pFacialModelAlignment[count] = nullptr;
		m_pFacialModelBuilder[count] = nullptr;
		m_bFacialModelProduced[count] = false;
		m_pFacialModel[count] = nullptr;
	}

	LARGE_INTEGER qpf = {0};
	if (QueryPerformanceFrequency(&qpf))
	{
		m_fFreq = double(qpf.QuadPart);
	}
}

NuiKinectV2Manager::~NuiKinectV2Manager()
{
	ShutdownDevice ();
}

bool NuiKinectV2Manager::InitializeColorFrame()
{
	assert(m_pNuiSensor);
	if(!m_pNuiSensor)
		return false;

	HRESULT hResult = S_OK;
	// Source
	IColorFrameSource* pColorSource;
	hResult = m_pNuiSensor->get_ColorFrameSource( &pColorSource );
	if( SUCCEEDED( hResult ) )
	{
		// Description
		IFrameDescription* pColorDescription;
		hResult = pColorSource->get_FrameDescription( &pColorDescription );
		if( SUCCEEDED( hResult ) )
		{
			int nColorWidth = 0;
			int nColorHeight = 0;
			pColorDescription->get_Width( &nColorWidth ); // 1920
			pColorDescription->get_Height( &nColorHeight ); // 1080
			m_colorFrameBuffer.AllocateBuffer(nColorWidth, nColorHeight);

			// Reader
			/*hResult = pColorSource->OpenReader( &m_pColorFrameReader );
			if( FAILED( hResult ) ){
				std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
			}*/
		}
		else
		{
			std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
		}
		SafeRelease( pColorDescription );
	}
	else
	{
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
	}
	SafeRelease( pColorSource );
	return SUCCEEDED( hResult );
}

bool NuiKinectV2Manager::InitializeDepthFrame()
{
	assert(m_pNuiSensor);
	if(!m_pNuiSensor)
		return false;

	HRESULT hResult = S_OK;
	// Source
	IDepthFrameSource* pDepthSource;
	hResult = m_pNuiSensor->get_DepthFrameSource( &pDepthSource );
	if( SUCCEEDED( hResult ) )
	{
		// Description
		IFrameDescription* pDepthDescription;
		hResult = pDepthSource->get_FrameDescription( &pDepthDescription );
		if( SUCCEEDED( hResult ) )
		{
			int nDepthWidth = 0;
			int nDepthHeight = 0;
			pDepthDescription->get_Width( &nDepthWidth ); // 512
			pDepthDescription->get_Height( &nDepthHeight ); // 424
			m_depthFrameBuffer.AllocateBuffer(nDepthWidth, nDepthHeight);

			UINT16 minDepth, maxDepth;
			if( SUCCEEDED(pDepthSource->get_DepthMinReliableDistance(&minDepth)) )
				m_depthFrameBuffer.SetMinDepth(minDepth);
			if( SUCCEEDED(pDepthSource->get_DepthMaxReliableDistance(&maxDepth)) )
				m_depthFrameBuffer.SetMaxDepth(maxDepth);

			// Reader
			/*hResult = pDepthSource->OpenReader( &m_pDepthFrameReader );
			if( FAILED( hResult ) ){
				std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
			}*/
		}
		else
		{
			std::cerr << "Error : IDepthFrameSource::get_FrameDescription()" << std::endl;
		}
		SafeRelease( pDepthDescription );
	}
	else
	{
		std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
	}
	SafeRelease( pDepthSource );

	return SUCCEEDED( hResult );
}

bool NuiKinectV2Manager::InitializeBodyIndexFrame()
{
	assert(m_pNuiSensor);
	if(!m_pNuiSensor)
		return false;

	HRESULT hResult = S_OK;
	// Source
	IBodyIndexFrameSource* pBodyIndexSource;
	hResult = m_pNuiSensor->get_BodyIndexFrameSource( &pBodyIndexSource );
	if( SUCCEEDED( hResult ) && pBodyIndexSource)
	{
		// Description
		IFrameDescription* pBodyIndexDescription;
		hResult = pBodyIndexSource->get_FrameDescription( &pBodyIndexDescription );
		if( SUCCEEDED( hResult ) && pBodyIndexDescription)
		{
			int nBodyIndexWidth = 0;
			int nBodyIndexHeight = 0;
			pBodyIndexDescription->get_Width( &nBodyIndexWidth ); // 512
			pBodyIndexDescription->get_Height( &nBodyIndexHeight ); // 424
			m_bodyIndexFrameBuffer.AllocateBuffer(nBodyIndexWidth, nBodyIndexHeight);
			// Reader
			/*hResult = pBodyIndexSource->OpenReader( &m_pBodyIndexFrameReader );
			if( FAILED( hResult ) ){
				std::cerr << "Error : IBodyIndexFrameSource::OpenReader()" << std::endl;
			}*/
		}
		else
		{
			std::cerr << "Error : IDepthFrameSource::get_FrameDescription()" << std::endl;
		}
		SafeRelease( pBodyIndexDescription );
	}
	else
	{
		std::cerr << "Error : IKinectSensor::get_BodyIndexFrameSource()" << std::endl;
	}
	SafeRelease( pBodyIndexSource );
	return (S_OK == hResult);
}

bool NuiKinectV2Manager::LoadGestureDatabase(const wchar_t* databaseFileName)
{
	assert(databaseFileName);
	if(!databaseFileName)
		return false;

	// Create Gesture Dataase from File (*.gba)
	IVisualGestureBuilderDatabase* pGestureDatabase = nullptr;
	HRESULT hResult = CreateVisualGestureBuilderDatabaseInstanceFromFile( databaseFileName, &pGestureDatabase );
	if( FAILED( hResult ) ){
		SafeRelease( pGestureDatabase );
		std::cerr << "Error : CreateVisualGestureBuilderDatabaseInstanceFromFile()" << std::endl;
		return false;
	}

	// Add Gesture
	UINT nGestureCount = 0;
	hResult = pGestureDatabase->get_AvailableGesturesCount( &nGestureCount );
	if( FAILED( hResult ) || (0 == nGestureCount) ){
		SafeRelease( pGestureDatabase );
		std::cerr << "Error : IVisualGestureBuilderDatabase::get_AvailableGesturesCount()" << std::endl;
		return false;
	}

	std::cout << "There are " << nGestureCount << " gestures in the database: " << std::endl;
	IGesture** aGestureList = new IGesture*[nGestureCount];
	hResult = pGestureDatabase->get_AvailableGestures( nGestureCount, aGestureList );
	if( FAILED( hResult ) || (0 == nGestureCount) ){
		SafeRelease( pGestureDatabase );
		SafeDeleteArray( aGestureList );
		std::cerr << "Error : IVisualGestureBuilderDatabase::get_AvailableGesturesCount()" << std::endl;
		return false;
	}
	SafeRelease( pGestureDatabase );

	for (UINT i = 0; i < nGestureCount; ++i)
	{
		m_aGestureArray.push_back(aGestureList[i]);
	}
	SafeDeleteArray( aGestureList );
	return (S_OK == hResult);
}

bool NuiKinectV2Manager::InitializeGestureSource()
{
	assert(m_pNuiSensor);
	if(!m_pNuiSensor)
		return false;

	// release gesture data
	for (UINT i = 0; i < m_aGestureArray.size(); ++i)
	{
		IGesture* pGesture = m_aGestureArray.at(i);
		if(pGesture)
			pGesture->Release();
	}
	m_aGestureArray.clear();
	m_aGestureTypeMap.clear();

	std::wstring sDatabaseFile1 = L"C:\\Users\\ME\\Documents\\Kinect Studio\\Repository\\RightHandUp.gba";
	LoadGestureDatabase( sDatabaseFile1.c_str() );
	std::wstring sDatabaseFile2 = L"C:\\Users\\ME\\Documents\\Kinect Studio\\Repository\\RightHandUpToDown.gba";
	LoadGestureDatabase( sDatabaseFile2.c_str() );
	/*std::wstring sDatabaseFile3 = L"OpenHand.gba";
	LoadGestureDatabase( sDatabaseFile3.c_str() );*/

	GestureType mType;
	const UINT uTextLength = 260; // magic number, if value smaller than 260, can't get name
	wchar_t sName[uTextLength];
	UINT nGestureCount = (UINT)m_aGestureArray.size();
	for (UINT iGest = 0; iGest < nGestureCount; ++iGest)
	{
		IGesture* pGesture = m_aGestureArray.at(iGest);
		if(!pGesture)
			continue;
		if (pGesture->get_GestureType(&mType) == S_OK)
		{
			if (mType == GestureType_Discrete)
				std::cout << "\t[D] ";
			else if (mType == GestureType_Continuous)
				std::cout << "\t[C] ";

			if (pGesture->get_Name(uTextLength, sName) == S_OK)
				std::wcout << sName << std::endl;

			wchar_t headUp[] = L"RightHandUp";
			wchar_t swipe[] = L"RightHandUpToDown";
			wchar_t openHand[] = L"OpenHand";
			if (std::wcscmp(sName,headUp) == 0)
			{
				m_aGestureTypeMap.insert(GESTURE_TYPE_INDEX_MAP::value_type(iGest, Gesture_Discrete_HandUp));
			}
			else if (std::wcscmp(sName,swipe) == 0)
			{
				m_aGestureTypeMap.insert(GESTURE_TYPE_INDEX_MAP::value_type(iGest, Gesture_Continuous_Swipe));
			}
			else if (std::wcscmp(sName,openHand) == 0)
			{
				m_aGestureTypeMap.insert(GESTURE_TYPE_INDEX_MAP::value_type(iGest, Gesture_Discrete_OpenHand));
			}
		}
	}
	
	HRESULT hResult = S_OK;
	for( UINT count = 0; count < BODY_COUNT; count++ ){
		// Source
		hResult = CreateVisualGestureBuilderFrameSource( m_pNuiSensor, 0, &m_pGestureSources[count] );
		if( FAILED( hResult ) ){
			std::cerr << "Error : CreateVisualGestureBuilderFrameSource()" << std::endl;
			break;
		}

		hResult = m_pGestureSources[count]->AddGestures(nGestureCount, m_aGestureArray.data());
		if( FAILED( hResult ) ){
			std::cerr << "Error : IVisualGestureBuilderFrameSource::AddGestures()" << std::endl;
			break;
		}

		for (UINT iGest = 0; iGest < nGestureCount; ++iGest)
		{
			hResult = m_pGestureSources[count]->SetIsEnabled( m_aGestureArray.at(iGest), true );
			if( FAILED( hResult ) ){
				std::cerr << "Error : IVisualGestureBuilderFrameSource::SetIsEnabled()" << std::endl;
				break;
			}
		}

		// Reader
		hResult = m_pGestureSources[count]->OpenReader( &m_pGestureReaders[count] );
		if( FAILED( hResult ) ){
			std::cerr << "Error : IVisualGestureBuilderFrameSource::OpenReader()" << std::endl;
			break;
		}
	}

	return (S_OK == hResult);
}

bool NuiKinectV2Manager::InitializeFaceTracking()
{
	assert(m_pNuiSensor);
	if(!m_pNuiSensor)
		return false;

	DWORD features = FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
		| FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
		| FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
		| FaceFrameFeatures::FaceFrameFeatures_Happy
		| FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
		| FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
		| FaceFrameFeatures::FaceFrameFeatures_MouthOpen
		| FaceFrameFeatures::FaceFrameFeatures_MouthMoved
		| FaceFrameFeatures::FaceFrameFeatures_LookingAway
		| FaceFrameFeatures::FaceFrameFeatures_Glasses
		| FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;

	HRESULT hResult = S_OK;
	// create a face frame source + reader to track each body in the fov
	for (UINT i = 0; i < BODY_COUNT; i++)
	{
		// create the face frame source by specifying the required face frame features
		hResult = CreateFaceFrameSource(m_pNuiSensor, 0, features, &m_pFaceFrameSources[i]);
		if (FAILED(hResult))
		{
			std::cerr << "Error : CreateFaceFrameSource()" << std::endl;
			break;
		}
		// open the corresponding reader
		hResult = m_pFaceFrameSources[i]->OpenReader(&m_pFaceFrameReaders[i]);
		if (FAILED(hResult))
		{
			std::cerr << "Error : Open Face Reader Failed." << std::endl;
			break;
		}
	}
	return (S_OK == hResult);
}

bool NuiKinectV2Manager::InitializeFacialModel()
{
	assert(m_pNuiSensor);
	if(!m_pNuiSensor)
		return false;

	HRESULT hResult = S_OK;
	for( int count = 0; count < BODY_COUNT; count++ ){
		// Source
		hResult = CreateHighDefinitionFaceFrameSource( m_pNuiSensor, &m_pFacialModelSources[count] );
		if( FAILED( hResult ) ){
			std::cerr << "Error : CreateHighDefinitionFaceFrameSource()" << std::endl;
			break;
		}

		// Reader
		hResult = m_pFacialModelSources[count]->OpenReader( &m_pFacialModelReaders[count] );
		if( FAILED( hResult ) ){
			std::cerr << "Error : IHighDefinitionFaceFrameSource::OpenReader()" << std::endl;
			break;
		}

		// Open Face Model Builder
		hResult = m_pFacialModelSources[count]->OpenModelBuilder( FaceModelBuilderAttributes::FaceModelBuilderAttributes_None, &m_pFacialModelBuilder[count] );
		if( FAILED( hResult ) ){
			std::cerr << "Error : IHighDefinitionFaceFrameSource::OpenModelBuilder()" << std::endl;
			break;
		}

		// Start Collection Face Data
		hResult = m_pFacialModelBuilder[count]->BeginFaceDataCollection();
		if( FAILED( hResult ) ){
			std::cerr << "Error : IFaceModelBuilder::BeginFaceDataCollection()" << std::endl;
			break;
		}

		// Create Face Alignment
		hResult = CreateFaceAlignment( &m_pFacialModelAlignment[count] );
		if( FAILED( hResult ) ){
			std::cerr << "Error : CreateFaceAlignment()" << std::endl;
			break;
		}

		m_bFacialModelProduced[count] = false;
		m_aFacialModelDeformations[count].resize( FaceShapeDeformations::FaceShapeDeformations_Count );
		// Create Face Model
		hResult = CreateFaceModel( 1.0f, FaceShapeDeformations::FaceShapeDeformations_Count, &m_aFacialModelDeformations[count][0], &m_pFacialModel[count] );
		if( FAILED( hResult ) ){
			std::cerr << "Error : CreateFaceModel()" << std::endl;
			break;
		}
	}
	
	GetFaceModelVertexCount( &m_nFacialModelVertexNum ); // 1347

	return (S_OK == hResult);
}

bool NuiKinectV2Manager::InitializeDevice(DWORD initializeFlag)
{
	HRESULT hResult = S_OK;
	if(!m_pNuiSensor)
	{
		hResult = GetDefaultKinectSensor( &m_pNuiSensor );
		if( FAILED( hResult ) ){
			std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
			return false;
		}
	}

	assert(m_pNuiSensor);

	BOOLEAN bIsOpen = false;
	if((S_OK == m_pNuiSensor->get_IsOpen(&bIsOpen)) && bIsOpen )
	{
		BOOLEAN bIsPaused = false;
		if ( m_pMultiSourceFrameReader && (S_OK == m_pMultiSourceFrameReader->get_IsPaused(&bIsPaused)) && !bIsPaused )
		{
			std::cerr << "Can't re-initialize sensor. The sensor should be shut down first." << std::endl;
			return false;
		}
	}
	else
	{
		std::cout << "Try to open sensor" << std::endl;
		if (m_pNuiSensor->Open() != S_OK)
		{
			std::cerr << "Can't open sensor" << std::endl;
			return false;
		}
	}

	m_eDeviceFlags = EDevice_None;
	if(initializeFlag & EDevice_ColorMap_On)
		initializeFlag |= EDevice_Depth_On;
	if(initializeFlag & EDevice_CameraMap_On)
		initializeFlag |= EDevice_Depth_On;
	if(initializeFlag & EDevice_DepthMap_On)
		initializeFlag |= EDevice_Depth_On | EDevice_Color_On;

	DWORD enabledFrameSourceTypes = FrameSourceTypes::FrameSourceTypes_None;
	if(initializeFlag & EDevice_Depth_On)
	{
		if(InitializeDepthFrame())
		{
			m_eDeviceFlags |= EDevice_Depth_On;
			enabledFrameSourceTypes |= FrameSourceTypes::FrameSourceTypes_Depth;
		}
	}
	if(initializeFlag & EDevice_Color_On)
	{
		if(InitializeColorFrame())
		{
			m_eDeviceFlags |= EDevice_Color_On;
			enabledFrameSourceTypes |= FrameSourceTypes::FrameSourceTypes_Color;
		}
	}
	if(initializeFlag & EDevice_PlayIndex_On)
	{
		if(InitializeBodyIndexFrame())
		{
			m_eDeviceFlags |= EDevice_PlayIndex_On;
			enabledFrameSourceTypes |= FrameSourceTypes::FrameSourceTypes_BodyIndex;
		}
	}
	if(initializeFlag & EDevice_Skeleton_On)
	{
		m_eDeviceFlags |= EDevice_Skeleton_On;
		enabledFrameSourceTypes |= FrameSourceTypes::FrameSourceTypes_Body;
	}
	if(initializeFlag & EDevice_Gesture_On)
	{
		if(m_eDeviceFlags & EDevice_Skeleton_On)
		{
			if( InitializeGestureSource() )
				m_eDeviceFlags |= EDevice_Gesture_On;
		}
		else
		{
			std::cerr << "Error : Cannot initialize the gesture because the body capture is disabled." << std::endl;
		}
	}
	if(initializeFlag & EDevice_Face_On)
	{
		if(m_eDeviceFlags & EDevice_Skeleton_On)
		{
			if( InitializeFaceTracking() )
				m_eDeviceFlags |= EDevice_Face_On;
		}
		else
		{
			std::cerr << "Error : Cannot initialize the face tracking because the body capture is disabled." << std::endl;
		}
	}
	if(initializeFlag & EDevice_FacialModel_On)
	{
		if(m_eDeviceFlags & EDevice_Skeleton_On)
		{
			if( InitializeFacialModel() )
				m_eDeviceFlags |= EDevice_FacialModel_On;
		}
		else
		{
			std::cerr << "Error : Cannot initialize the facial model because the body capture is disabled." << std::endl;
		}
	}
	
	assert(!m_pMultiSourceFrameReader);
	hResult = m_pNuiSensor->OpenMultiSourceFrameReader(
		enabledFrameSourceTypes, &m_pMultiSourceFrameReader);
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::OpenMultiSourceFrameReader()" << std::endl;
		return false;
	}

	{
		// Coordinate Mapper
		hResult = m_pNuiSensor->get_CoordinateMapper( &m_pCoordinateMapper );
		if( FAILED( hResult ) ){
			std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
			//return false;
		}
		else
		{
			if(initializeFlag & EDevice_ColorMap_On)
				m_eDeviceFlags |= EDevice_ColorMap_On;
			if(initializeFlag & EDevice_CameraMap_On)
				m_eDeviceFlags |= EDevice_CameraMap_On;
			if(initializeFlag & EDevice_DepthMap_On)
				m_eDeviceFlags |= EDevice_DepthMap_On;
			if(initializeFlag & EDevice_Camera_Intrisics)
				m_eDeviceFlags |= EDevice_Camera_Intrisics;
		}
		hResult = m_pCoordinateMapper->SubscribeCoordinateMappingChanged(&m_coordinateMappingChangedEvent);
		if (FAILED(hResult))
		{
			std::cerr << "Error : IKinectSensor::SubscribeCoordinateMappingChanged()" << std::endl;
		}
		/*if(m_pCoordinateMapper)
		{
			CameraIntrinsics depthCameraIntri;
			hResult = m_pCoordinateMapper->GetDepthCameraIntrinsics(&depthCameraIntri);
		}*/

		SafeDeleteArray( m_pDepthDistortionMap );
	}

	return true;
}

bool NuiKinectV2Manager::IsDeviceOn() const
{
	if(!m_pNuiSensor)
		return false;

	BOOLEAN bIsAvailable = false;
	if(FAILED(m_pNuiSensor->get_IsAvailable(&bIsAvailable)) || !bIsAvailable)
	{
		std::cerr << "Error : The sensor is not available." << std::endl;
		return false;
	}

	BOOLEAN bIsOpen = false;
	if((S_OK != m_pNuiSensor->get_IsOpen(&bIsOpen)) || !bIsOpen )
		return false;
	BOOLEAN bIsPaused = false;
	return ( m_pMultiSourceFrameReader && (S_OK == m_pMultiSourceFrameReader->get_IsPaused(&bIsPaused)) && !bIsPaused );
}

bool NuiKinectV2Manager::StartDevice ()
{
	assert(m_pNuiSensor);
	/*BOOLEAN bIsAvailable = false;
	if(FAILED(m_pNuiSensor->get_IsAvailable(&bIsAvailable)) || !bIsAvailable)
	{
		std::cerr << "Error : The sensor is not available." << std::endl;
		return false;
	}*/
	BOOLEAN bIsOpen = false;
	if(!m_pNuiSensor || (S_OK != m_pNuiSensor->get_IsOpen(&bIsOpen)) || !bIsOpen)
	{
		std::cerr << "Can't start sensor since it's not initialized." << std::endl;
		return false;
	}

	assert(m_pMultiSourceFrameReader);
	if(!m_pMultiSourceFrameReader)
	{
		std::cerr << "Can't acquire the multiply source frame reader." << std::endl;
		return false;
	}
	HRESULT hResult = m_pMultiSourceFrameReader->put_IsPaused(false);
	if( FAILED( hResult ) ){
		std::cerr << "Error : IMultiSourceFrameReader::put_IsPaused(false)" << std::endl;
		return false;
	}

	// Subscribe Handle
	if(!m_hWaitableHandle)
	{
		hResult = m_pMultiSourceFrameReader->SubscribeMultiSourceFrameArrived( &m_hWaitableHandle );
		if( FAILED( hResult ) ){
			std::cerr << "Error : IColorFrameReader::SubscribeFrameArrived()" << std::endl;
			return false;
		}
	}

	if(!m_hThNuiProcess)
	{
		m_hEvNuiProcessStop=CreateEvent(NULL,TRUE,FALSE,NULL);
		m_hThNuiProcess = CreateThread(NULL,0,GrabThread,this,0,NULL);
		//m_Thread = std::thread(std::bind(&NuiKinectV2Manager::GrabThread, this));
	}
	return true;
}

void NuiKinectV2Manager::PauseDevice ()
{
	BOOLEAN bIsAvailable = false;
	if(!m_pNuiSensor || FAILED(m_pNuiSensor->get_IsAvailable(&bIsAvailable)) || !bIsAvailable)
	{
		return;
	}
	BOOLEAN bIsOpen = false;
	if((S_OK != m_pNuiSensor->get_IsOpen(&bIsOpen)) || !bIsOpen)
	{
		std::cout << "Can't pause sensor since it's not opened." << std::endl;
		return;
	}

	assert(m_pMultiSourceFrameReader);
	if(!m_pMultiSourceFrameReader)
	{
		std::cerr << "Can't acquire the multiply source frame reader." << std::endl;
		return;
	}

	HRESULT hResult = m_pMultiSourceFrameReader->put_IsPaused(true);
	if( FAILED( hResult ) ){
		std::cerr << "Error : IMultiSourceFrameReader::put_IsPaused()" << std::endl;
	}
	/*if(m_hWaitableHandle)
	{
		hResult = m_pMultiSourceFrameReader->UnsubscribeMultiSourceFrameArrived( m_hWaitableHandle );
		if( FAILED( hResult ) ){
			std::cerr << "Error : IMultiSourceFrameReader::UnsubscribeMultiSourceFrameArrived()" << std::endl;
		}
		m_hWaitableHandle = NULL;
	}*/

	// Stop the Nui processing thread
	if(m_hEvNuiProcessStop!=NULL)
	{
		// Signal the thread
		SetEvent(m_hEvNuiProcessStop);

		// Wait for thread to stop
		if(m_hThNuiProcess!=NULL)
		{
			WaitForSingleObject(m_hThNuiProcess,INFINITE);
			CloseHandle(m_hThNuiProcess);
			m_hThNuiProcess = NULL;
		}
		CloseHandle(m_hEvNuiProcessStop);
		m_hEvNuiProcessStop = NULL;
	}
	//m_Thread.join();
}

void NuiKinectV2Manager::ShutdownDevice()
{
	// Stop the Nui processing thread
	if(m_hEvNuiProcessStop!=NULL)
	{
		// Signal the thread
		SetEvent(m_hEvNuiProcessStop);

		// Wait for thread to stop
		if(m_hThNuiProcess!=NULL)
		{
			WaitForSingleObject(m_hThNuiProcess,INFINITE);
			CloseHandle(m_hThNuiProcess);
			m_hThNuiProcess = NULL;
		}
		CloseHandle(m_hEvNuiProcessStop);
		m_hEvNuiProcessStop = NULL;
	}
	//m_Thread.join();

	if(m_pMultiSourceFrameReader && m_hWaitableHandle)
	{
		m_pMultiSourceFrameReader->UnsubscribeMultiSourceFrameArrived( m_hWaitableHandle );
		m_hWaitableHandle = NULL;
	}
	//CloseHandle(m_hWaitableHandle);

	// release frame reader
	SafeRelease( m_pMultiSourceFrameReader );

	// release buffer
	SafeDeleteArray( m_pDepthDistortionMap );

	// release coordinate mapper
	if (nullptr != m_pCoordinateMapper && m_coordinateMappingChangedEvent)
	{
		m_pCoordinateMapper->UnsubscribeCoordinateMappingChanged(m_coordinateMappingChangedEvent);
		m_coordinateMappingChangedEvent = NULL;
	}
	SafeRelease( m_pCoordinateMapper );

	// release gesture data
	m_aGestureTypeMap.clear();
	for (UINT i = 0; i < m_aGestureArray.size(); ++i)
	{
		IGesture* pGesture = m_aGestureArray.at(i);
		if(pGesture)
			pGesture->Release();
	}
	m_aGestureArray.clear();
	for( UINT count = 0; count < BODY_COUNT; count++ ){
		SafeRelease( m_pGestureSources[count] );
		SafeRelease( m_pGestureReaders[count] );
		SafeRelease( m_pFaceFrameSources[count] );
		SafeRelease( m_pFaceFrameReaders[count] );
		SafeRelease( m_pFacialModelSources[count] );
		SafeRelease( m_pFacialModelReaders[count] );
		SafeRelease( m_pFacialModelAlignment[count] );
		SafeRelease( m_pFacialModelBuilder[count] );
		SafeRelease( m_pFacialModel[count] );
		m_bFacialModelProduced[count] = false;
	}
	m_nFacialModelVertexNum = 0;

	// Close and Release Sensor
	if( m_pNuiSensor ){
		m_pNuiSensor->Close();
	}
	SafeRelease( m_pNuiSensor );

	m_depthFrameBuffer.Clear();
	m_colorFrameBuffer.Clear();
	m_bodyIndexFrameBuffer.Clear();
	m_compoundImage.Clear();
}

bool NuiKinectV2Manager::OnCoordinateMappingChanged()
{
	// Calculate the down sampled image sizes, which are used for the AlignPointClouds calculation frames
	CameraIntrinsics intrinsics = {};

	m_pCoordinateMapper->GetDepthCameraIntrinsics(&intrinsics);

	if (m_cameraIntri.m_fx == intrinsics.FocalLengthX &&
		m_cameraIntri.m_fy == intrinsics.FocalLengthY &&
		m_cameraIntri.m_cx == intrinsics.PrincipalPointX &&
		m_cameraIntri.m_cy == intrinsics.PrincipalPointY)
		return true;

	m_cameraIntri.m_fx = intrinsics.FocalLengthX;
	m_cameraIntri.m_fy = intrinsics.FocalLengthY;
	m_cameraIntri.m_cx = intrinsics.PrincipalPointX;
	m_cameraIntri.m_cy = intrinsics.PrincipalPointY;

	const UINT cDepthWidth = m_depthFrameBuffer.GetWidth();
	const UINT cDepthHeight = m_depthFrameBuffer.GetHeight();
	const UINT cDepthImagePixels = cDepthWidth * cDepthHeight;

	float focalLengthX = intrinsics.FocalLengthX / cDepthWidth;
	float focalLengthY = intrinsics.FocalLengthY / cDepthHeight;
	float principalPointX = intrinsics.PrincipalPointX / cDepthWidth;
	float principalPointY = intrinsics.PrincipalPointY / cDepthHeight;

	if (nullptr == m_pDepthDistortionMap)
	{
		m_pDepthDistortionMap = new(std::nothrow) DepthSpacePoint[cDepthImagePixels];
	}
	if (nullptr == m_pDepthDistortionMap)
	{
		std::cerr << "Failed to initialize Kinect Fusion depth image distortion buffer." << std::endl;
		return false;
	}

	if (principalPointX != 0)
	{
		HRESULT hr = E_UNEXPECTED;

		CameraSpacePoint cameraFrameCorners[4] = //at 1 meter distance. Take into account that depth frame is mirrored
		{
			/*LT*/{ -principalPointX / focalLengthX, principalPointY / focalLengthY, 1.f },
			/*RT*/{ (1.f - principalPointX) / focalLengthX, principalPointY / focalLengthY, 1.f },
			/*LB*/{ -principalPointX / focalLengthX, (principalPointY - 1.f) / focalLengthY, 1.f },
			/*RB*/{ (1.f - principalPointX) / focalLengthX, (principalPointY - 1.f) / focalLengthY, 1.f }
		};

		CameraSpacePoint* cameraCoordsRow = new CameraSpacePoint[cDepthWidth];

		for (UINT rowID = 0; rowID < cDepthHeight; rowID++)
		{
			const float rowFactor = float(rowID) / float(cDepthHeight - 1);
			const CameraSpacePoint rowStart =
			{
				cameraFrameCorners[0].X + (cameraFrameCorners[2].X - cameraFrameCorners[0].X) * rowFactor,
				cameraFrameCorners[0].Y + (cameraFrameCorners[2].Y - cameraFrameCorners[0].Y) * rowFactor,
				1.f
			};

			const CameraSpacePoint rowEnd =
			{
				cameraFrameCorners[1].X + (cameraFrameCorners[3].X - cameraFrameCorners[1].X) * rowFactor,
				cameraFrameCorners[1].Y + (cameraFrameCorners[3].Y - cameraFrameCorners[1].Y) * rowFactor,
				1.f
			};

			const float stepFactor = 1.f / float(cDepthWidth - 1);
			const CameraSpacePoint rowDelta =
			{
				(rowEnd.X - rowStart.X) * stepFactor,
				(rowEnd.Y - rowStart.Y) * stepFactor,
				0
			};

			CameraSpacePoint currentPoint = rowStart;
			for (UINT i = 0; i < cDepthWidth; i++)
			{
				cameraCoordsRow[i] = currentPoint;
				currentPoint.X += rowDelta.X;
				currentPoint.Y += rowDelta.Y;
			}

			hr = m_pCoordinateMapper->MapCameraPointsToDepthSpace(cDepthWidth, cameraCoordsRow, cDepthWidth, &m_pDepthDistortionMap[rowID * cDepthWidth]);
			if (FAILED(hr))
			{
				break;
			}
		}
		SafeDeleteArray(cameraCoordsRow);

		if (FAILED(hr))
		{
			std::cerr << "Failed to initialize Kinect Coordinate Mapper." << std::endl;
			return false;
		}

		UINT* pDepthDistortionLT = m_depthDistortionFrameBuffer.AllocateBuffer(cDepthWidth, cDepthHeight);
		if (!pDepthDistortionLT)
		{
			std::cerr << "Failed to initialize Kinect Fusion depth image distortion Lookup Table." << std::endl;
			return false;
		}

		for (UINT i = 0; i < cDepthImagePixels; i++, pDepthDistortionLT++)
		{
			//nearest neighbor depth lookup table 
			UINT x = UINT(m_pDepthDistortionMap[i].X + 0.5f);
			UINT y = UINT(m_pDepthDistortionMap[i].Y + 0.5f);

			*pDepthDistortionLT = (x < cDepthWidth && y < cDepthHeight) ? x + y * cDepthWidth : UINT_MAX;
		}
		
	}
	return true;
}

/*static*/
DWORD WINAPI NuiKinectV2Manager::GrabThread (LPVOID pParam)
{
	NuiKinectV2Manager*  pthis=(NuiKinectV2Manager *) pParam;
	assert(pthis);
	HANDLE hEvents[ ] = { reinterpret_cast<HANDLE>( pthis->m_hWaitableHandle ), pthis->m_hEvNuiProcessStop };

	while (true)
	{
		if (!pthis->m_pBuffer)
			continue;
		
		// Waitable Events
		WaitForMultipleObjects( ARRAYSIZE( hEvents ), hEvents, false, INFINITE );

		// If the stop event is set, stop looping and exit
		if (WAIT_OBJECT_0 == WaitForSingleObject(pthis->m_hEvNuiProcessStop, 0))
		{
			break;
		}

		std::shared_ptr<NuiCompositeFrame> pCompositeFrame = pthis->m_pBuffer->allocateFrame();
		if (!pCompositeFrame)
			continue;

		pthis->GrabFrame(pCompositeFrame.get());

		if ((pthis->m_eDeviceFlags & EDevice_Camera_Intrisics)
			&& pthis->m_coordinateMappingChangedEvent != NULL)
		{
			pCompositeFrame->m_cameraParams.m_intrinsics = pthis->m_cameraIntri;

			if (pthis->m_eDeviceFlags & EDevice_Depth_On)
			{
				if (WAIT_OBJECT_0 == WaitForSingleObject((HANDLE)pthis->m_coordinateMappingChangedEvent, 0))
				{
					pthis->OnCoordinateMappingChanged();
					ResetEvent((HANDLE)pthis->m_coordinateMappingChangedEvent);
				}
				pCompositeFrame->m_depthDistortionFrame.WriteFrameLock();
				pCompositeFrame->m_depthDistortionFrame = pthis->m_depthDistortionFrameBuffer;
				pCompositeFrame->m_depthDistortionFrame.WriteFrameUnlock();
			}
		}
		
		if(pthis->m_eDeviceFlags & EDevice_CameraMap_On)
		{
			if( !pthis->GrabDepthToCameraMap(pCompositeFrame.get()) )
			{
				std::cerr << "Error : GrabDepthToCameraMap()" << std::endl;
			}
		}
		if(pthis->m_eDeviceFlags & EDevice_ColorMap_On)
		{
			/*if (std::abs(pthis->m_depthFrameBuffer.GetTimeStamp() - pthis->m_colorFrameBuffer.GetTimeStamp()) < cHalfADepthFrameMs) */
			if( !pthis->GrabDepthToColorMap(pCompositeFrame.get()) )
			{
				std::cerr << "Error : GrabDepthToCameraMap()" << std::endl;
			}
		}
		if(pthis->m_eDeviceFlags & EDevice_DepthMap_On)
		{
			/*if (std::abs(pthis->m_depthFrameBuffer.GetTimeStamp() - pthis->m_colorFrameBuffer.GetTimeStamp()) < cHalfADepthFrameMs) */
			if( !pthis->GrabColorToDepthMap(pCompositeFrame.get()) )
			{
				std::cerr << "Error : GrabColorToDepthMap()" << std::endl;
			}
		}

		if(pthis->m_eDeviceFlags & EDevice_Face_On)
		{
			pthis->DetectFaces(pCompositeFrame.get());
		}
		if(pthis->m_eDeviceFlags & EDevice_Gesture_On)
		{
			pthis->DetectGestures(pCompositeFrame.get());
		}
		if(pthis->m_eDeviceFlags & EDevice_FacialModel_On)
		{
			pthis->DetectFacialModels(pCompositeFrame.get());
		}

		//std::this_thread::sleep_for((std::chrono::milliseconds(10)));
	}
	return 0;
}

bool NuiKinectV2Manager::GrabFrame (NuiCompositeFrame* pCompositeFrame)
{
	assert(pCompositeFrame);
	if(!pCompositeFrame)
		return false;

#ifdef _DEBUG
	LARGE_INTEGER tGrabStartTimeStamp = {0};
	QueryPerformanceCounter(&tGrabStartTimeStamp);
#endif

	// Arrived Data
	IMultiSourceFrameArrivedEventArgs* pMultiArgs = nullptr;
	HRESULT hResult = m_pMultiSourceFrameReader->GetMultiSourceFrameArrivedEventData( m_hWaitableHandle, &pMultiArgs );
	if( SUCCEEDED( hResult ) ){
		// Reference
		IMultiSourceFrameReference* pMultiReference = nullptr;
		hResult = pMultiArgs->get_FrameReference( &pMultiReference );
		if( SUCCEEDED( hResult ) ){
			// Frame
			IMultiSourceFrame* pMultiSourceFrame = nullptr;
			hResult = pMultiReference->AcquireFrame( &pMultiSourceFrame );
			if( SUCCEEDED( hResult ) )
			{
				//Read depth data
				if(m_eDeviceFlags & EDevice_Depth_On)
				{
					if( GrabDepthFrame(pMultiSourceFrame) )
					{
						pCompositeFrame->m_depthFrame.WriteFrameLock();
						pCompositeFrame->m_depthFrame = m_depthFrameBuffer;
						pCompositeFrame->m_depthFrame.WriteFrameUnlock();
					}
				}

				//Read color data
				if(m_eDeviceFlags & EDevice_Color_On)
				{
					if( GrabColorFrame(pMultiSourceFrame) )
					{
						pCompositeFrame->m_colorFrame.WriteFrameLock();
						pCompositeFrame->m_colorFrame = m_colorFrameBuffer;
						pCompositeFrame->m_colorFrame.WriteFrameUnlock();
					}
				}

				//Read Body Index data
				if(m_eDeviceFlags & EDevice_PlayIndex_On)
				{
					if( GrabBodyIndexFrame(pMultiSourceFrame) )
					{
						pCompositeFrame->m_bodyIndexFrame.WriteFrameLock();
						pCompositeFrame->m_bodyIndexFrame = m_bodyIndexFrameBuffer;
						pCompositeFrame->m_bodyIndexFrame.WriteFrameUnlock();
					}
				}

				//Read Body data
				if(m_eDeviceFlags & EDevice_Skeleton_On)
				{
					GrabBodyFrame(pMultiSourceFrame, pCompositeFrame);
				}
			}
			SafeRelease( pMultiSourceFrame );
		}
		SafeRelease( pMultiReference );
	}
	SafeRelease( pMultiArgs );

#ifdef _DEBUG
	LARGE_INTEGER tGrabEndTimeStamp = {0};
	QueryPerformanceCounter(&tGrabEndTimeStamp);
	if(m_fFreq)
	{
		double grabTime = double(tGrabEndTimeStamp.QuadPart - tGrabStartTimeStamp.QuadPart) / m_fFreq;
		//std::cout << "KinectV2 Grab Time : " << grabTime << std::endl;
	}
#endif
	return SUCCEEDED( hResult );
}

bool NuiKinectV2Manager::GrabDepthFrame (IMultiSourceFrame* pMultiSourceFrame)
{
	assert(pMultiSourceFrame);
	if(!pMultiSourceFrame)
		return false;

	IDepthFrameReference* pDepthFrameReference = nullptr;
	HRESULT hResult = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
	if (SUCCEEDED(hResult) && pDepthFrameReference)
	{
		IDepthFrame* pDepthFrame = NULL;
		hResult = pDepthFrameReference->AcquireFrame(&pDepthFrame);
		if (SUCCEEDED(hResult) && pDepthFrame)
		{
			UINT16*	pDepthBuffer = m_depthFrameBuffer.GetBuffer();
			size_t nDepthBufferNum = m_depthFrameBuffer.GetBufferNum();
			if(pDepthBuffer && nDepthBufferNum > 0)
			{
				hResult = pDepthFrame->CopyFrameDataToArray((UINT)nDepthBufferNum, pDepthBuffer);
				if(SUCCEEDED(hResult))
				{
					// Compute FPS
					TIMESPAN relativeTimeStamp;
					pDepthFrame->get_RelativeTime(&relativeTimeStamp);
					m_depthFrameBuffer.SetTimeStamp(relativeTimeStamp);
					m_nDepthIntervalCount ++;
					if (m_nDepthIntervalCount > cFPStimeInterval)
					{
						TIMESPAN span = relativeTimeStamp - m_lastDepthTimeStamp;
						m_depthFrameBuffer.SetFPS( (double)m_nDepthIntervalCount * 1.0e7f / (double)span + 0.5 );
						m_lastDepthTimeStamp = relativeTimeStamp;
						m_nDepthIntervalCount = 0;
#ifdef _PRINT_FPS
						std::cout << "Kinect depth fps: " << m_depthFrameBuffer.GetFPS() << std::endl;
#endif
					}
				}
			}
			else
			{
				hResult = S_FALSE;
				std::cerr << "Error : Cannot grab the depth frame since its buffer is not allocated." << std::endl;
			}
		}
		SafeRelease(pDepthFrame);
	}
	SafeRelease(pDepthFrameReference);

	return SUCCEEDED(hResult);
}

bool NuiKinectV2Manager::GrabColorFrame (IMultiSourceFrame* pMultiSourceFrame)
{
	assert(pMultiSourceFrame);
	if(!pMultiSourceFrame)
		return false;

	IColorFrameReference* pColorFrameReference = NULL;
	HRESULT hResult = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
	if (SUCCEEDED(hResult) && pColorFrameReference)
	{
		IColorFrame* pColorFrame = NULL;
		hResult = pColorFrameReference->AcquireFrame(&pColorFrame);
		if (SUCCEEDED(hResult) && pColorFrame)
		{
			BGRQUAD* pColorBuffer = m_colorFrameBuffer.GetBuffer();
			size_t nColorBufferSize = m_colorFrameBuffer.GetBufferSize();
			if(pColorBuffer && nColorBufferSize > 0)
			{
				/*IColorCameraSettings* pCameraSettings = NULL;
				pColorFrame->get_ColorCameraSettings(&pCameraSettings);
				if (pCameraSettings)
				{
					TIMESPAN exposureTime;
					pCameraSettings->get_ExposureTime(&exposureTime);
				}*/
				hResult = pColorFrame->CopyConvertedFrameDataToArray((UINT)nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
				if(SUCCEEDED(hResult))
				{
					// Compute FPS
					TIMESPAN relativeTimeStamp;
					pColorFrame->get_RelativeTime(&relativeTimeStamp);
					m_colorFrameBuffer.SetTimeStamp(relativeTimeStamp);
					m_nColorIntervalCount ++;
					if (m_nColorIntervalCount > cFPStimeInterval)
					{
						TIMESPAN span = relativeTimeStamp - m_lastColorTimeStamp;
						m_colorFrameBuffer.SetFPS( (double)m_nColorIntervalCount * 1.0e7f / (double)span + 0.5 );
						m_lastColorTimeStamp = relativeTimeStamp;
						m_nColorIntervalCount = 0;
					}
				}
			}
			else
			{
				hResult = S_FALSE;
				std::cerr << "Error : Cannot grab the color frame since its buffer is not allocated." << std::endl;
			}
		}
		SafeRelease(pColorFrame);
	}
	SafeRelease(pColorFrameReference);

	return SUCCEEDED(hResult);
}

bool NuiKinectV2Manager::GrabBodyIndexFrame (IMultiSourceFrame* pMultiSourceFrame)
{
	assert(pMultiSourceFrame);
	if(!pMultiSourceFrame)
		return false;

	IBodyIndexFrameReference* pBodyIndexFrameReference = NULL;
	HRESULT hResult = pMultiSourceFrame->get_BodyIndexFrameReference(&pBodyIndexFrameReference);
	if (SUCCEEDED(hResult) && pBodyIndexFrameReference)
	{
		IBodyIndexFrame* pBodyIndexFrame = NULL;
		hResult = pBodyIndexFrameReference->AcquireFrame(&pBodyIndexFrame);
		if (SUCCEEDED(hResult) && pBodyIndexFrame)
		{
			BYTE* pBodyIndexBuffer = m_bodyIndexFrameBuffer.GetBuffer();
			size_t nBodyIndexBufferSize = m_bodyIndexFrameBuffer.GetBufferSize();
			if(pBodyIndexBuffer && nBodyIndexBufferSize > 0)
			{
				hResult = pBodyIndexFrame->CopyFrameDataToArray((UINT)nBodyIndexBufferSize, pBodyIndexBuffer);
				if(SUCCEEDED(hResult))
				{
					TIMESPAN relativeTimeStamp;
					pBodyIndexFrame->get_RelativeTime(&relativeTimeStamp);
					m_bodyIndexFrameBuffer.SetTimeStamp(relativeTimeStamp);
				}
			}
			else
			{
				hResult = S_FALSE;
				std::cerr << "Error : Cannot grab the body index frame since its buffer is not allocated." << std::endl;
			}
		}
		SafeRelease(pBodyIndexFrame);
	}
	SafeRelease(pBodyIndexFrameReference);
	
	return SUCCEEDED(hResult);
}

bool NuiKinectV2Manager::GrabBodyFrame (IMultiSourceFrame* pMultiSourceFrame, NuiCompositeFrame* pCompositeFrame)
{
	assert(pMultiSourceFrame);
	if(!pMultiSourceFrame)
		return false;

	bool bNeedCache = false;
	if(m_pBuffer && pCompositeFrame)
	{
		bNeedCache = true;
	}
	IBodyFrameReference* pBodyFrameReference = NULL;
	HRESULT hResult = pMultiSourceFrame->get_BodyFrameReference(&pBodyFrameReference);
	if (SUCCEEDED(hResult))
	{
		IBodyFrame* pBodyFrame = NULL;
		hResult = pBodyFrameReference->AcquireFrame(&pBodyFrame);
		if (SUCCEEDED(hResult))
		{
			IBody* ppBodies[BODY_COUNT] = {0};
			hResult = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
			if (SUCCEEDED(hResult))
			{
				for (UINT iBody = 0; iBody < BODY_COUNT; ++iBody)
				{
					IBody* pBody = ppBodies[iBody];
					if (pBody)
					{
						NuiSkeletonJoints* pSkeletonJoints = nullptr;

						BOOLEAN bTracked = false;
						hResult = pBody->get_IsTracked(&bTracked);
						if (SUCCEEDED(hResult) && bTracked)
						{
							// Set TrackingID to Detect Gesture & Face
							UINT64 uTrackingId = _UI64_MAX;
							hResult = pBody->get_TrackingId( &uTrackingId );
							if( SUCCEEDED( hResult ) ){
								// get tracking id of gesture
								UINT64 uGestureId = 0;
								if (m_pGestureSources[iBody] && m_pGestureSources[iBody]->get_TrackingId(&uGestureId) == S_OK)
								{
									if (uGestureId != uTrackingId)
									{
										// assign traking ID if the value is changed
										std::cout << "Gesture Source " << iBody << " start to track user " << uTrackingId << std::endl;
										m_pGestureSources[iBody]->put_TrackingId(uTrackingId);
									}
								}
								// get tracking id of face
								UINT64 uFaceId = 0;
								if (m_pFaceFrameSources[iBody] && m_pFaceFrameSources[iBody]->get_TrackingId(&uFaceId) == S_OK)
								{
									if (uFaceId != uTrackingId)
									{
										// assign traking ID if the value is changed
										std::cout << "Face Source " << iBody << " start to track user " << uTrackingId << std::endl;
										m_pFaceFrameSources[iBody]->put_TrackingId(uTrackingId);
									}
								}
								// get tracking id of facial model
								UINT64 uFacialModelId = 0;
								if (m_pFacialModelSources[iBody] && m_pFacialModelSources[iBody]->get_TrackingId(&uFacialModelId) == S_OK)
								{
									if (uFacialModelId != uTrackingId)
									{
										// assign traking ID if the value is changed
										std::cout << "Face Source " << iBody << " start to track user " << uTrackingId << std::endl;
										m_pFacialModelSources[iBody]->put_TrackingId(uTrackingId);
									}
								}
							}

							if(bNeedCache)
							{
								pSkeletonJoints = new NuiSkeletonJoints;

								HandState leftHandState = HandState_Unknown;
								TrackingConfidence confidence = TrackingConfidence_Low;
								pBody->get_HandLeftConfidence(&confidence);
								if( SUCCEEDED(pBody->get_HandLeftState(&leftHandState)) )
								{
									pSkeletonJoints->SetLeftHand(leftHandState);
								}

								HandState rightHandState = HandState_Unknown;
								confidence = TrackingConfidence_Low;
								pBody->get_HandRightConfidence(&confidence);
								if( SUCCEEDED(pBody->get_HandRightState(&rightHandState)) )
								{
									pSkeletonJoints->SetRightHand(rightHandState);
								}
								
								Joint joints[JointType_Count];
								hResult = pBody->GetJoints(_countof(joints), joints);
								if (SUCCEEDED(hResult))
								{
									for (UINT j = 0; j < _countof(joints); ++j)
									{
										if (joints[j].TrackingState == TrackingState_Tracked)
										{
											// For debug
											/*if(j == JointType_HandLeft)
											{
												std::cerr << "In grab: " << joints[j].Position.X << "\t" << joints[j].Position.Y << " \t" << joints[j].Position.Z << std::endl;
											}*/
											pSkeletonJoints->SetJoint3DPos((NuiJointType)j, joints[j].Position.X, joints[j].Position.Y, joints[j].Position.Z);
											if(m_pCoordinateMapper)
											{
												ColorSpacePoint ptInCS;
												hResult = m_pCoordinateMapper->MapCameraPointToColorSpace(joints[j].Position, &ptInCS);
												if (SUCCEEDED(hResult))
													pSkeletonJoints->SetJoint2DPos((NuiJointType)j, ptInCS.X, ptInCS.Y);
											}
										}
										else if(joints[j].TrackingState == TrackingState_Inferred)
										{
											pSkeletonJoints->SetJoint3DPos((NuiJointType)j, joints[j].Position.X, joints[j].Position.Y, joints[j].Position.Z, true);
											if(m_pCoordinateMapper)
											{
												ColorSpacePoint ptInCS;
												hResult = m_pCoordinateMapper->MapCameraPointToColorSpace(joints[j].Position, &ptInCS);
												if (SUCCEEDED(hResult))
													pSkeletonJoints->SetJoint2DPos((NuiJointType)j, ptInCS.X, ptInCS.Y, true);
											}
										}
									}
								}
							}
						}
						if(bNeedCache)
						{
							if( !pCompositeFrame->m_skeletonFrame.CacheSkeleton(iBody, pSkeletonJoints))
								SafeDelete(pSkeletonJoints);
							TIMESPAN relativeTimeStamp;
							if(S_OK == pBodyFrame->get_RelativeTime(&relativeTimeStamp))
								pCompositeFrame->m_skeletonFrame.SetTimeStamp(relativeTimeStamp);
						}
					}
				}
			}
			for (UINT i = 0; i < _countof(ppBodies); ++i)
			{
				SafeRelease(ppBodies[i]);
			}
		}
		SafeRelease(pBodyFrame);
	}
	SafeRelease(pBodyFrameReference);

	return SUCCEEDED(hResult);
}

bool NuiKinectV2Manager::GrabDepthToCameraMap (NuiCompositeFrame* pCompositeFrame)
{
	assert(pCompositeFrame);
	if(!pCompositeFrame)
		return false;

	assert(m_pCoordinateMapper);
	if(!m_pCoordinateMapper)
		return false;

	UINT16* pDepthBuffer = m_depthFrameBuffer.GetBuffer();
	if(!pDepthBuffer)
		return false;

	const UINT nDepthWidth = m_depthFrameBuffer.GetWidth();
	const UINT nDepthHeight = m_depthFrameBuffer.GetHeight();
	const UINT nDepthPointNum = nDepthWidth * nDepthHeight;
	if(0 == nDepthPointNum)
		return false;

	// map to camera space
	CameraSpacePoint* pDepthToCamera = pCompositeFrame->m_cameraMapFrame.AllocateBuffer(nDepthWidth, nDepthHeight);
	if(!pDepthToCamera)
		return false;
	pCompositeFrame->m_cameraMapFrame.WriteFrameLock();
	HRESULT hResult = m_pCoordinateMapper->MapDepthFrameToCameraSpace( nDepthPointNum, pDepthBuffer, nDepthPointNum, pDepthToCamera);
	pCompositeFrame->m_cameraMapFrame.WriteFrameUnlock();

	pCompositeFrame->m_cameraMapFrame.SetTimeStamp(m_depthFrameBuffer.GetTimeStamp());
	return SUCCEEDED(hResult);
}

bool NuiKinectV2Manager::GrabDepthToColorMap (NuiCompositeFrame* pCompositeFrame)
{
	assert(pCompositeFrame);
	if(!pCompositeFrame)
		return false;

	assert(m_pCoordinateMapper);
	if(!m_pCoordinateMapper)
		return false;

	UINT16* pDepthBuffer = m_depthFrameBuffer.GetBuffer();
	if(!pDepthBuffer)
		return false;

	const UINT nDepthWidth = m_depthFrameBuffer.GetWidth();
	const UINT nDepthHeight = m_depthFrameBuffer.GetHeight();
	const UINT nDepthPointNum = nDepthWidth * nDepthHeight;
	if(0 == nDepthPointNum)
		return false;

	// map to color space
	ColorSpacePoint* pDepthToColor = pCompositeFrame->m_colorMapFrame.AllocateBuffer(nDepthWidth, nDepthHeight);
	if(!pDepthToColor)
		return false;
	pCompositeFrame->m_colorMapFrame.WriteFrameLock();
	HRESULT hResult = m_pCoordinateMapper->MapDepthFrameToColorSpace(nDepthPointNum, pDepthBuffer, nDepthPointNum, pDepthToColor);
	pCompositeFrame->m_colorMapFrame.WriteFrameUnlock();

	pCompositeFrame->m_colorMapFrame.SetTimeStamp(m_depthFrameBuffer.GetTimeStamp());
	return SUCCEEDED(hResult);
}

bool NuiKinectV2Manager::GrabColorToDepthMap (NuiCompositeFrame* pCompositeFrame)
{
	assert(pCompositeFrame);
	if(!pCompositeFrame)
		return false;

	assert(m_pCoordinateMapper);
	if(!m_pCoordinateMapper)
		return false;

	UINT16* pDepthBuffer = m_depthFrameBuffer.GetBuffer();
	if(!pDepthBuffer)
		return false;

	const UINT nDepthWidth = m_depthFrameBuffer.GetWidth();
	const UINT nDepthHeight = m_depthFrameBuffer.GetHeight();
	const UINT nDepthPointNum = nDepthWidth * nDepthHeight;
	if(0 == nDepthPointNum)
		return false;

	const UINT nColorWidth = m_colorFrameBuffer.GetWidth();
	const UINT nColorHeight = m_colorFrameBuffer.GetHeight();
	const UINT nColorPointNum = nColorWidth * nColorHeight;
	if(0 == nColorPointNum)
		return false;

	// color to depth space
	DepthSpacePoint* pColorToDepth = pCompositeFrame->m_depthMapFrame.AllocateBuffer(nColorWidth, nColorHeight);
	if(!pColorToDepth)
		return false;
	pCompositeFrame->m_depthMapFrame.WriteFrameLock();
	HRESULT hResult = m_pCoordinateMapper->MapColorFrameToDepthSpace( nDepthPointNum, pDepthBuffer, nColorPointNum, pColorToDepth);
	pCompositeFrame->m_depthMapFrame.WriteFrameUnlock();

	pCompositeFrame->m_depthMapFrame.SetTimeStamp(m_depthFrameBuffer.GetTimeStamp());
	return SUCCEEDED(hResult);
}

bool NuiKinectV2Manager::DetectGestures(NuiCompositeFrame* pCompositeFrame)
{
#ifdef _DEBUG
	LARGE_INTEGER tProcessStartTimeStamp = {0};
	QueryPerformanceCounter(&tProcessStartTimeStamp);
#endif
	bool bNeedCache = false;
	if(m_pBuffer && pCompositeFrame)
	{
		bNeedCache = true;
	}

	HRESULT hResult = S_OK;
	for( UINT count = 0; count < BODY_COUNT; count++ )
	{
		if(!m_pGestureReaders[count])
			continue;

		IVisualGestureBuilderFrame* pGestureFrame = nullptr;
		hResult = m_pGestureReaders[count]->CalculateAndAcquireLatestFrame( &pGestureFrame );
		if( SUCCEEDED( hResult ) && pGestureFrame != nullptr )
		{
			NuiGestureResult* pGestureResponse = nullptr;
			BOOLEAN bGestureTracked = false;
			hResult = pGestureFrame->get_IsTrackingIdValid( &bGestureTracked );
			if( SUCCEEDED( hResult ) && bGestureTracked )
			{
				GestureType mType;
				const UINT uTextLength = 260;
				wchar_t sName[uTextLength];
				
				pGestureResponse = new NuiGestureResult;
				// for each gestures
				UINT nGestureCount = (UINT)m_aGestureArray.size();
				for (UINT j = 0; j < nGestureCount; ++j)
				{
					IGesture* pGesture = m_aGestureArray.at(j);
					if(!pGesture)
						continue;
					// get gesture information
					pGesture->get_GestureType(&mType);
					pGesture->get_Name(uTextLength, sName);

					UINT typeIndex;
					GESTURE_TYPE_INDEX_MAP::iterator typeIter = m_aGestureTypeMap.find(j);
					if(typeIter != m_aGestureTypeMap.end())
					{
						typeIndex = typeIter->second;
					}
					else
					{
						std::cerr << "Unknown Gesture Type: " << sName << std::endl;
						continue;
					}
					if (mType == GestureType_Discrete)
					{
						// Discrete Gesture
						NuiDiscreteGestureType actionType = (NuiDiscreteGestureType)typeIndex;
						NuiDiscreteGestureResult discreteResult;
						discreteResult.bDetected = false;
						discreteResult.fConfidence = 0.0f;

						IDiscreteGestureResult* pGestureResult = nullptr;
						hResult = pGestureFrame->get_DiscreteGestureResult( pGesture, &pGestureResult );
						if( SUCCEEDED( hResult ) && pGestureResult != nullptr )
						{
							// check if is detected
							BOOLEAN bDetected = false;
							hResult = pGestureResult->get_Detected( &bDetected );
							if( SUCCEEDED( hResult ) && bDetected )
							{
								float fConfidence = 0.0f;
								pGestureResult->get_Confidence(&fConfidence);

								discreteResult.bDetected = true;
								discreteResult.fConfidence = fConfidence;
								pGestureResponse->SetDiscreteGestureResult(actionType, discreteResult);
								std::cout << "Detected Gesture" << sName << L" @" << fConfidence << std::endl;
							}
						}
						SafeRelease( pGestureResult );

						pGestureResponse->SetDiscreteGestureResult(actionType, discreteResult);
					}
					else if (mType == GestureType_Continuous)
					{
						// Continuous Gesture (Sample Swipe.gba is Action to Swipe the hand in horizontal direction.)
						NuiContinuousGestureType actionType = (NuiContinuousGestureType)typeIndex;
						NuiContinuousGestureResult continuousResult;
						continuousResult.fProgress = 0.0f;

						IContinuousGestureResult* pGestureResult = nullptr;
						hResult = pGestureFrame->get_ContinuousGestureResult( pGesture, &pGestureResult );
						if( SUCCEEDED( hResult ) && pGestureResult != nullptr )
						{
							hResult = pGestureResult->get_Progress( &continuousResult.fProgress );
							if( SUCCEEDED( hResult ) && continuousResult.fProgress > 0.5f ){
								std::cout << "Progress: " << sName << " " << continuousResult.fProgress << std::endl;
							}
						}
						SafeRelease( pGestureResult );

						pGestureResponse->SetContinuousGestureResult(actionType, continuousResult);
					}
				}
			}
			// Cache
			if(bNeedCache)
			{
				if( !pCompositeFrame->m_gestureFrame.CacheGestureResult(count, pGestureResponse) )
					SafeDelete(pGestureResponse);
				TIMESPAN relativeTimeStamp;
				if(S_OK == pGestureFrame->get_RelativeTime(&relativeTimeStamp))
					pCompositeFrame->m_gestureFrame.SetTimeStamp(relativeTimeStamp);
			}
			else
			{
				SafeDelete(pGestureResponse);
			}
		}
		SafeRelease( pGestureFrame );
	}
#ifdef _DEBUG
	LARGE_INTEGER tProcessEndTimeStamp = {0};
	QueryPerformanceCounter(&tProcessEndTimeStamp);
	if(m_fFreq)
	{
		double processTime = double(tProcessEndTimeStamp.QuadPart - tProcessStartTimeStamp.QuadPart) / m_fFreq;
		//std::cout << "KinectV2 Gesture Detect Time : " << processTime << std::endl;
	}
#endif
	return SUCCEEDED(hResult);
}

bool NuiKinectV2Manager::DetectFaces(NuiCompositeFrame* pCompositeFrame)
{
	assert(pCompositeFrame);
	if(!pCompositeFrame)
		return false;

#ifdef _DEBUG
	LARGE_INTEGER tProcessStartTimeStamp = {0};
	QueryPerformanceCounter(&tProcessStartTimeStamp);
#endif
	HRESULT hResult = E_FAIL;
	for( UINT iFace = 0; iFace < BODY_COUNT; iFace++ )
	{
		if(!m_pFaceFrameReaders[iFace])
			continue;

		// retrieve the latest face frame from this reader
		IFaceFrame* pFaceFrame = nullptr;
		hResult = m_pFaceFrameReaders[iFace]->AcquireLatestFrame(&pFaceFrame);
		if( SUCCEEDED( hResult ) && pFaceFrame != nullptr )
		{
			NuiTrackedFace* pTrackedFace = nullptr;
			BOOLEAN bFaceTracked = false;
			// check if a valid face is tracked in this face frame
			hResult = pFaceFrame->get_IsTrackingIdValid(&bFaceTracked);
			if( SUCCEEDED( hResult ) && bFaceTracked )
			{
				IFaceFrameResult* pFaceFrameResult = nullptr;
				hResult = pFaceFrame->get_FaceFrameResult(&pFaceFrameResult);
				// need to verify if pFaceFrameResult contains data before trying to access it
				if (SUCCEEDED(hResult) && pFaceFrameResult != nullptr)
				{
					pTrackedFace = new NuiTrackedFace;
					// Face Bounding Box
					RectI faceBox = {0};
					hResult = pFaceFrameResult->get_FaceBoundingBoxInColorSpace(&faceBox);
					if (SUCCEEDED(hResult))
					{
						pTrackedFace->SetBoundingBox(faceBox);
					}

					// Face Point
					PointF facePoints[FacePointType::FacePointType_Count];
					hResult = pFaceFrameResult->GetFacePointsInColorSpace(FacePointType::FacePointType_Count, facePoints);
					if (SUCCEEDED(hResult))
					{
						for (UINT iType = FacePointType::FacePointType_EyeLeft; iType < FacePointType::FacePointType_Count; ++iType)
						{
							pTrackedFace->SetFace2DPoint(iType, facePoints[iType]);
						}
					}

					// Face Rotation
					Vector4 faceRotation;
					hResult = pFaceFrameResult->get_FaceRotationQuaternion(&faceRotation);
					if (SUCCEEDED(hResult))
					{
						pTrackedFace->SetRotationQuaternion(faceRotation);
					}

					// Face Property
					DetectionResult faceProperties[FaceProperty::FaceProperty_Count];
					hResult = pFaceFrameResult->GetFaceProperties(FaceProperty::FaceProperty_Count, faceProperties);
					if (SUCCEEDED(hResult))
					{
						for (UINT iType = FaceProperty::FaceProperty_Happy; iType < FaceProperty::FaceProperty_Count; ++iType)
						{
							pTrackedFace->SetFaceProperties(iType, faceProperties[iType]);
						}
					}
				}
				SafeRelease(pFaceFrameResult);
			}
			// Cache
			if( !pCompositeFrame->m_faceTrackingFrame.CacheTrackedFace(iFace, pTrackedFace) )
				SafeDelete(pTrackedFace);
			TIMESPAN relativeTimeStamp;
			if(S_OK == pFaceFrame->get_RelativeTime(&relativeTimeStamp))
				pCompositeFrame->m_faceTrackingFrame.SetTimeStamp(relativeTimeStamp);
		}
		SafeRelease( pFaceFrame );
	}
#ifdef _DEBUG
	LARGE_INTEGER tProcessEndTimeStamp = {0};
	QueryPerformanceCounter(&tProcessEndTimeStamp);
	if(m_fFreq)
	{
		double processTime = double(tProcessEndTimeStamp.QuadPart - tProcessStartTimeStamp.QuadPart) / m_fFreq;
		//std::cout << "KinectV2 Face Tracking Time : " << processTime << std::endl;
	}
#endif

	return SUCCEEDED(hResult);
}

bool NuiKinectV2Manager::DetectFacialModels(NuiCompositeFrame* pCompositeFrame)
{
	assert(pCompositeFrame);
	if(!pCompositeFrame)
		return false;

#ifdef _DEBUG
	LARGE_INTEGER tProcessStartTimeStamp = {0};
	QueryPerformanceCounter(&tProcessStartTimeStamp);
#endif

	HRESULT hResult = E_FAIL;
	for( int count = 0; count < BODY_COUNT; count++ )
	{
		if(!m_pFacialModelReaders[count])
			continue;

		IHighDefinitionFaceFrame* pHDFaceFrame = nullptr;
		hResult = m_pFacialModelReaders[count]->AcquireLatestFrame( &pHDFaceFrame );
		if( SUCCEEDED( hResult ) && pHDFaceFrame != nullptr )
		{
			NuiFacialModel* pFacialModelCache = nullptr;
			BOOLEAN bFaceTracked = false;
			hResult = pHDFaceFrame->get_IsFaceTracked( &bFaceTracked );
			if( SUCCEEDED( hResult ) && bFaceTracked )
			{
				hResult = pHDFaceFrame->GetAndRefreshFaceAlignmentResult( m_pFacialModelAlignment[count] );
				if( SUCCEEDED( hResult ) && m_pFacialModelAlignment[count] != nullptr )
				{
					// Face Model Building
					if( !m_bFacialModelProduced[count] )
					{
						FaceModelBuilderCollectionStatus collection;
						hResult = m_pFacialModelBuilder[count]->get_CollectionStatus( &collection );
						//if( collection == FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_Complete )
						{
							std::cout << "Status : Complete" << std::endl;
							printf("Status : Complete");

							IFaceModelData* pFaceModelData = nullptr;
							hResult = m_pFacialModelBuilder[count]->GetFaceData( &pFaceModelData );
							if( SUCCEEDED( hResult ) && pFaceModelData != nullptr ){
								hResult = pFaceModelData->ProduceFaceModel( &m_pFacialModel[count] );
								if( SUCCEEDED( hResult ) && m_pFacialModel[count] != nullptr ){
									m_bFacialModelProduced[count] = true;
								}
							}
							SafeRelease( pFaceModelData );
						}
						//else
						{
							std::cout << "Status : " << collection << std::endl;
							
							// Collection Status
							if( collection >= FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_TiltedUpViewsNeeded ){
								std::cout << "Need : Tilted Up Views" << std::endl;
							}
							else if( collection >= FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_RightViewsNeeded ){
								std::cout << "Need : Right Views" << std::endl;
							}
							else if( collection >= FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_LeftViewsNeeded ){
								std::cout << "Need : Left Views" << std::endl;
							}
							else if( collection >= FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_FrontViewFramesNeeded ){
								std::cout << "Need : Front ViewFrames" << std::endl;
							}

							// Capture Status
							FaceModelBuilderCaptureStatus capture;
							hResult = m_pFacialModelBuilder[count]->get_CaptureStatus( &capture );
							switch( capture ){
							case FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_FaceTooFar:
								std::cout << "Error : Face Too Far from Camera" << std::endl;
								break;
							case FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_FaceTooNear:
								std::cout << "Error : Face Too Near to Camera" << std::endl;
								break;
							case FaceModelBuilderCaptureStatus_MovingTooFast:
								std::cout << "Error : Moving Too Fast" << std::endl;
								break;
							default:
								break;
							}
						}
					}

					pFacialModelCache = new NuiFacialModel;
					hResult = m_pFacialModelAlignment[count]->GetAnimationUnits(FaceShapeAnimations::FaceShapeAnimations_Count, pFacialModelCache->AllocateAUs(FaceShapeAnimations::FaceShapeAnimations_Count));

					//hResult = m_pFacialModel[count]->GetFaceShapeDeformations( FaceShapeDeformations::FaceShapeDeformations_Count, &m_aFacialModelDeformations[count][0] );
					// HD Face Points
					//hResult = m_pFacialModel[count]->CalculateVerticesForAlignment( m_pFacialModelAlignment[count], m_nFacialModelVertexNum, pFacialModelCache->AllocateVertices(m_nFacialModelVertexNum) );
					if(FAILED(hResult))
						SafeDelete(pFacialModelCache);
				}
			}
			// Cache
			if( !pCompositeFrame->m_facialModelFrame.CacheFacialModel(count, pFacialModelCache) )
				SafeDelete(pFacialModelCache);
			TIMESPAN relativeTimeStamp;
			if(S_OK == pHDFaceFrame->get_RelativeTime(&relativeTimeStamp))
				pCompositeFrame->m_faceTrackingFrame.SetTimeStamp(relativeTimeStamp);
		}
		SafeRelease( pHDFaceFrame );
	}

#ifdef _DEBUG
	LARGE_INTEGER tProcessEndTimeStamp = {0};
	QueryPerformanceCounter(&tProcessEndTimeStamp);
	if(m_fFreq)
	{
		double processTime = double(tProcessEndTimeStamp.QuadPart - tProcessStartTimeStamp.QuadPart) / m_fFreq;
		//std::cout << "KinectV2 Facial Model Time : " << processTime << std::endl;
	}
#endif

	return SUCCEEDED(hResult);
}
