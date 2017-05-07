#pragma once

#include <pangolin/pangolin.h>

// Forwards
class NuiCLMappableData;
class NuiPangoCloudShader;
class NuiPangoMeshShader;
class NuiPangoTexturedCloudShader;
class NuiPangoTexturedMeshShader;
class NuiPangoPolygonMeshShader;

class NuiPangoVis
{
public:
	NuiPangoVis(bool showcaseMode);
	~NuiPangoVis();

	void render(NuiCLMappableData* pData);

protected:
	void preCall();
	void postCall();
	void draw(NuiCLMappableData* pData);
	void updateView(NuiCLMappableData* pData);

	void drawFrustum(NuiCLMappableData* pData);
	void drawBoundingBox(NuiCLMappableData* pData);
	void displayImg(const std::string & id, NuiTextureMappable& texMappable);

public:
	pangolin::Var<bool> a_deviceOn;
	pangolin::Var<bool> a_start;
	pangolin::Var<bool> a_stepIn;
	pangolin::Var<bool> a_stop;

	pangolin::Var<bool> a_fileToFrame;
	pangolin::Var<bool> a_frameToFile;

	pangolin::Var<int> a_depthNearPlane;
	pangolin::Var<int> a_depthFarPlane;

	pangolin::Var<bool> a_kinFuOn;
	pangolin::Var<bool> a_kinFuPause;
	pangolin::Var<bool> a_trackColors;
	pangolin::Var<bool> a_hashingVolume;
	pangolin::Var<float> a_volumeVoxelSize;
	pangolin::Var<float> a_translateBasisX;
	pangolin::Var<float> a_translateBasisZ;
	pangolin::Var<float> a_integrationThreshold;

	pangolin::Var<bool> a_drawGlobalModel;
	pangolin::Var<bool> a_drawMesh;

	pangolin::Var<std::string> a_grabberSpeed;
	pangolin::Var<std::string> a_trackerFrameID;
	pangolin::Var<std::string> a_trackerSpeed;
	pangolin::Var<std::string> a_trackerLagFrames;
	pangolin::Var<std::string> a_saverLagFrames;
	pangolin::Var<std::string> a_totalPoints;

	pangolin::Var<float> a_confidenceThreshold;
	pangolin::Var<float> a_icpWeight;

	pangolin::DataLog a_resLog, a_inLog;

private:

	bool m_showcaseMode;

	NuiPangoTexturedCloudShader*	m_pTexturedCloudDraw;
	NuiPangoCloudShader*			m_pCloudDraw;
	NuiPangoTexturedMeshShader*		m_pTexturedMeshDraw;
	NuiPangoMeshShader*				m_pMeshDraw;
	NuiPangoPolygonMeshShader*		m_pPolygonMeshDraw;

	pangolin::Var<float> a_drawPointSize;
	pangolin::Var<bool> a_followPose;
	pangolin::Var<bool> a_drawFrustum;
	pangolin::Var<bool> a_drawBoundingBox;
	pangolin::Var<bool> a_drawColorImage;
	pangolin::Var<bool> a_drawNormals;
	pangolin::Var<bool> a_drawGraph;

	pangolin::Var<int> a_gpuMem;

	pangolin::Plotter *a_trackErrorPlot;

	pangolin::OpenGlRenderState s_cam;
};
