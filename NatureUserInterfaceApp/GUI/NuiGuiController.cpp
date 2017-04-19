#include "NuiGuiController.h"

#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"
#include "OpenCLUtilities/NuiOpenCLBufferFactory.h"
#include "OpenCLUtilities/NuiOfflineRenderFactory.h"
#include "NuiGuiOpenCLUtilities.h"
#include "NuiGuiHWMappable.h"
#include "NuiGuiHWTextureMappable.h"
#include "NuiPangoRayIntervalSplattingShader.h"

#include "DeviceManager/NuiRGBDDeviceController.h"

#include "NuiPangoVis.h"

#include "SLAM/VisualOdometry/NuiKinfuManager.h"
#include "Frame/Buffer/NuiFrameBuffer.h"
#include "Frame/NuiFrameSaveManager.h"
#include "Shape/NuiCLMappableData.h"
#include "Frame/NuiFrameUtilities.h"
#include "Foundation/NuiTimeLog.h"

static std::string sTestDataFolder = getenv("NUI_TESTDATA") ? getenv("NUI_TESTDATA") : "H:\\tmp\\";

NuiGuiController::NuiGuiController()
	: m_pCache(NULL)
	, m_pDevice(NULL)
	, m_gui(NULL)
	, m_pFrameToFile(NULL)
	, m_pKinfu(NULL)
{
	m_pCache = new NuiFrameBuffer();
	m_gui = new NuiPangoVis(false);
	m_pDevice = new NuiRGBDDeviceController();

	//Initialize OpenCL
	if( !NuiOpenCLGlobal::instance().isCLReady() )
	{
		if( !NuiOpenCLGlobal::instance().initializeOpenCL() )
			printf("Failed to initialize OpenCL.\n");

		// Register buffer functions, register more if needed in the future
		NuiOpenCLBufferFactory::RegisterAsUInt32IndexBufferCLFn(
			NuiGuiHWMappable::asUInt32IndexBufferCL);
		NuiOpenCLBufferFactory::RegisterAsPosition3fBufferCLFn(
			NuiGuiHWMappable::asPosition3fBufferCL);
		NuiOpenCLBufferFactory::RegisterAsColor4fBufferCLFn(
			NuiGuiHWMappable::asColor4fBufferCL);
		NuiOpenCLBufferFactory::RegisterAsNormal3fBufferCLFn(
			NuiGuiHWMappable::asNormal3fBufferCL);
		NuiOpenCLBufferFactory::RegisterAsPatchUV2fBufferCLFn(
			NuiGuiHWMappable::asTexture2fBufferCL);
		NuiOpenCLBufferFactory::RegisterAsTexture1fBufferCLFn(
			NuiGuiHWMappable::asTexture1fBufferCL);
		NuiOpenCLBufferFactory::RegisterAsTexture2DCLFn(
			NuiGuiHWTextureMappable::asHWTextureBufferSharedWithCL);
		NuiOpenCLBufferFactory::RegisterAsFrameTexture2DCLFn(
			NuiGuiHWTextureMappable::asFrameTextureBufferSharedWithCL);
		NuiOpenCLBufferFactory::RegisterAsRenderBufferCLFn(
			NuiGuiHWTextureMappable::asHWRenderBufferSharedWithCL);

		// Register functions for NuiGPUMemManager
		NuiGPUMemManager::RegisterInformRenderHoldGPU(NuiGuiOpenCLUtilities::informRenderHoldGPU);
		NuiGPUMemManager::RegisterInformRenderReleaseGPU(NuiGuiOpenCLUtilities::informRenderReleaseGPU);
		NuiGPUMemManager::RegisterLockRenderResourceHandle(NuiGuiOpenCLUtilities::lockResourceHandle);
		NuiGPUMemManager::RegisterUnlockRenderResourceHandle(NuiGuiOpenCLUtilities::unlockResourceHandle);

		NuiOfflineRenderFactory::RegisterInitializeOfflineRenderFn(
			NuiPangoRayIntervalSplattingShader::initializeShader);
		NuiOfflineRenderFactory::RegisterRunOfflineRenderFn(
			NuiPangoRayIntervalSplattingShader::render);

		NuiOpenCLGlobal::instance().isGL(true);
	}

}

NuiGuiController::~NuiGuiController()
{
	SafeDelete(m_pDevice);
	SafeDelete(m_pCache);
	SafeDelete(m_gui);
	SafeDelete(m_pFrameToFile);
	SafeDelete(m_pKinfu);
}

void NuiGuiController::resetCache()
{
	m_pCache->clear();
}

void NuiGuiController::handleGuiChanged()
{
	//assert(m_gui);

	if(m_gui->a_deviceOn.GuiChanged())
	{
		if(m_gui->a_deviceOn)
		{
			DWORD deviceMode = NuiRGBDDeviceController::EDeviceMode_VertexColorCamera;
			if(m_gui->a_fileToFrame)
				m_pDevice->startFileLoader(deviceMode, sTestDataFolder);
			else
				m_pDevice->startDevice(deviceMode);
		}
		else
		{
			m_pDevice->stopDevice();
		}
	}
	if(m_gui->a_frameToFile.GuiChanged())
	{
		if(m_gui->a_frameToFile)
		{
			if(!m_pFrameToFile)
				m_pFrameToFile = new NuiFrameSaveManager(sTestDataFolder);
			m_pFrameToFile->startThread();
		}
		else
		{
			SafeDelete(m_pFrameToFile);
		}
	}
	if(m_gui->a_kinFuOn.GuiChanged())
	{
		if(m_gui->a_kinFuOn && NuiOpenCLGlobal::instance().isCLReady())
		{
			if(!m_pKinfu)
			{
				m_pKinfu = new NuiKinfuManager();
			}

			m_pKinfu->pauseThread();
			m_pKinfu->m_engine.setColorTracker(m_gui->a_trackColors);
			m_pKinfu->m_engine.setTranslateBasis(Vector3f(m_gui->a_translateBasisX, 0.0f, m_gui->a_translateBasisZ));
			m_pKinfu->m_engine.setIntegrationMetricThreshold(m_gui->a_integrationThreshold);
			m_pKinfu->m_engine.setVolume(m_gui->a_volumeVoxelSize, m_gui->a_hashingVolume);
		}
		else
		{
			SafeDelete(m_pKinfu);
		}
	}
	if(m_gui->a_integrationThreshold.GuiChanged())
	{
		if(m_pKinfu)
			m_pKinfu->m_engine.setIntegrationMetricThreshold(m_gui->a_integrationThreshold);
	}

	/*if( NuiOpenCLGlobal::instance().isCLReady() )
	{
		NuiMeshingUtil::SmoothPositionCL(geomPtr);
		if( bHasNormals )
			NuiMeshingUtil::NormalEstimationCL(geomPtr);
	}*/

	/*if(pangolin::Pushed(m_gui->a_reset))
	{
		resetCache();
	}*/

	if(pangolin::Pushed(m_gui->a_start))
	{
		if(m_pKinfu)
			m_pKinfu->startThread();
	}
	else if(pangolin::Pushed(m_gui->a_stepIn))
	{
		if(m_pKinfu)
			m_pKinfu->stepIn();
	}
	else if(pangolin::Pushed(m_gui->a_stop))
	{
		if(m_pKinfu)
			m_pKinfu->stopThread();
	}
}

void NuiGuiController::writeGuiStatus(NuiCompositeFrame* pCompositeFrame)
{
	//assert(m_gui);
	if(!pCompositeFrame)
		return;

	if(m_pKinfu && m_pKinfu->isThreadOn())
	{
		float trackerErrThresh = 5e-05f;
		m_gui->a_resLog.Log(m_pKinfu->m_engine.getTrackerError(), trackerErrThresh);
		/*float trackerCountThresh = 30000.f;
		m_gui->a_inLog.Log((float)m_pKinfu->m_engine.getTrackerCount(), trackerCountThresh);*/
	}

	std::stringstream strsf;
	strsf << pCompositeFrame->m_depthFrame.GetFPS() << "Hz";
	m_gui->a_grabberSpeed = strsf.str();

	strsf.str("");
	if(m_pKinfu)
		strsf << m_pKinfu->m_engine.getFrameID();
	else
		strsf << 0;
	m_gui->a_trackerFrameID = strsf.str();

	strsf.str("");
	if(m_pKinfu)
		strsf << m_pKinfu->getLagFrames();
	else
		strsf << 0;
	m_gui->a_trackerLagFrames = strsf.str();

	strsf.str("");
	if(m_pFrameToFile)
		strsf << m_pFrameToFile->getLagFrames();
	else
		strsf << 0;
	m_gui->a_saverLagFrames = strsf.str();

	/*strsf.str("");
	strsf << NuiTimeLog::instance().avgFPS(NuiTimeLog::E_Time_Tracker);
	m_gui->a_trackerSpeed = strsf.str();*/
}

void NuiGuiController::readGuiStatus(NuiCompositeFrame* pCompositeFrame)
{
	//assert(m_gui);
	if(!pCompositeFrame)
		return;

	if(m_gui->a_depthNearPlane.GuiChanged())
	{
		pCompositeFrame->m_depthFrame.SetMinDepth(m_gui->a_depthNearPlane);
	}
	if(m_gui->a_depthFarPlane.GuiChanged())
	{
		pCompositeFrame->m_depthFrame.SetMaxDepth(m_gui->a_depthFarPlane);
	}

}

void NuiGuiController::launch()
{
	assert(m_gui);

	NuiCLMappableData frameData;

	while( !pangolin::ShouldQuit() )
	{
		handleGuiChanged();

		assert(m_pCache);
		if (!m_pCache)
			continue;

		if(m_pDevice)
		{
			do 
			{
				std::shared_ptr<NuiCompositeFrame> pFrame = m_pDevice->popFrame();
				if(pFrame)
				{
					readGuiStatus(pFrame.get());

					if(m_pFrameToFile && m_pFrameToFile->isThreadOn())
					{
						m_pFrameToFile->pushbackFrame(pFrame);
					}
					if(m_pKinfu /*&& m_pKinfu->isThreadOn()*/)
					{
						m_pKinfu->pushbackFrame(pFrame);
					}
					m_pCache->pushbackFrame(pFrame);
					pFrame.reset();
				}
				else
				{
					break;
				}
			} while (1);
		}

		std::shared_ptr<NuiCompositeFrame> pCurrentFrame = m_pCache->getLatestFrame();
		if(m_gui->a_drawGlobalModel && m_pKinfu /*&& m_pKinfu->isThreadOn()*/)
		{
			m_pKinfu->m_engine.getCLData(&frameData, m_gui->a_drawMesh);
		}
		else if(pCurrentFrame)
		{
			int indexFlags = m_gui->a_drawMesh ? NuiCLMappableData::E_MappableData_Triangle : NuiCLMappableData::E_MappableData_Point;
			NuiFrameUtilities::FrameToMappableData(pCurrentFrame.get(), &frameData, indexFlags, false, 0.2f);
		}

		m_gui->render(&frameData);

		writeGuiStatus(pCurrentFrame.get());
		// Total points
		std::stringstream strsf;
		size_t totalPoints = frameData.PointIndices().size();
		if(0 == totalPoints)
			totalPoints = frameData.TriangleIndices().size() / 3 + 2;
		if(0 == totalPoints)
			totalPoints = frameData.WireframeIndices().size() / 6 + 2;
		strsf << totalPoints;
		m_gui->a_totalPoints = strsf.str();

		pCurrentFrame.reset();
	}

	NuiTimeLog::instance().print();

	std::string fileName = sTestDataFolder + "\\Log\\";
	fileName.append("log");
	fileName.append(".txt");

	NuiFileIOUtilities::writeDayTime(fileName);
	if(m_pKinfu)
		m_pKinfu->m_engine.log(fileName);
	NuiTimeLog::instance().log(fileName);
}

