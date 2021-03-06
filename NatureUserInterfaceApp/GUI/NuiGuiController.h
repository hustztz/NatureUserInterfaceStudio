#pragma once

// Forwards
class NuiPangoVis;
class NuiRGBDDeviceController;
class NuiFrameCacheImpl;
class NuiCompositeFrame;
class NuiVisualFrameSaveManager;
namespace NuiSLAMEngine
{
	class NuiSLAMController;
}


class NuiGuiController
{
public:
	NuiGuiController();
	~NuiGuiController();

	void launch();

protected:
	void handleGuiChanged();
	void writeGuiStatus(NuiCompositeFrame* pCompositeFrame);
	void readGuiStatus(NuiCompositeFrame* pCompositeFrame);

	void resetCache();

private:
	NuiPangoVis*			m_gui;

	NuiRGBDDeviceController*	m_pDevice;
	NuiFrameCacheImpl*			m_pCache;
	NuiVisualFrameSaveManager*	m_pFrameToFile;
	NuiSLAMEngine::NuiSLAMController*	m_pSLAM;
};