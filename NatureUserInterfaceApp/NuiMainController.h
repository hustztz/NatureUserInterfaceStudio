#pragma once

// Forwards
class NuiPangoVis;
class NuiRGBDDeviceController;
class NuiFrameCacheImpl;
class NuiCompositeFrame;
class NuiFrameSaveManager;
class NuiKinfuManager;

class NuiMainController
{
public:
	NuiMainController();
	~NuiMainController();

	void launch();

protected:
	void handleGuiChanged();
	void writeGuiStatus(NuiCompositeFrame* pCompositeFrame);
	void readGuiStatus(NuiCompositeFrame* pCompositeFrame);

	void resetCache();

private:
	NuiPangoVis*			m_gui;

	NuiRGBDDeviceController*	m_pDevice;
	NuiFrameCacheImpl*		m_pCache;
	NuiFrameSaveManager*	m_pFrameToFile;
	NuiKinfuManager*		m_pKinfu;
};