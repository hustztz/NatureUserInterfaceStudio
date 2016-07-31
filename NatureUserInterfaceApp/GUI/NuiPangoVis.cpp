#include "NuiPangoVis.h"

#include "NuiPangoCloudShader.h"
#include "NuiPangoTexturedCloudShader.h"
#include "NuiPangoMeshShader.h"
#include "NuiPangoTexturedMeshShader.h"

#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>

#include "Shape/NuiCLMappableData.h"

#include <vector>

#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049

NuiPangoVis::NuiPangoVis(bool showcaseMode)
	: m_showcaseMode(showcaseMode)
	, m_pCloudDraw(NULL)
	, m_pTexturedCloudDraw(NULL)
	, m_pMeshDraw(NULL)
	, m_pTexturedMeshDraw(NULL)
	, a_deviceOn("ui.Device On", false, true)
	, a_pause("ui.Pause", false, true)
	, a_reset("ui.Reset", false, false)
	, a_fileToFrame("ui.Frame Loader On", false, true)
	, a_frameToFile("ui.Frame Saver On", false, true)
	, a_depthNearPlane("ui.Depth Near Plane", 400, 400, 4500)
	, a_depthFarPlane("ui.Depth Far Plane", 4200, 400, 4500)
	, a_drawFrustum("ui.Draw Frustum", false, true)
	, a_followPose("ui.Follow Pose", true, true)
	, a_drawPointSize("ui.Draw Point Size", 2.0f, 1.0f, 10.0f)
	, a_drawBoundingBox("ui.Draw Bounding Box", true, true)
	, a_drawColorImage("ui.Draw Color Image", true, true)
	, a_drawGlobalModel("ui.Draw Global Model", true, true)
	, a_drawMesh("ui.Draw Mesh", false, true)
	, a_drawNormals("ui.Draw Normals", false, true)
	, a_drawGraph("ui.Draw Graph", false, true)
	, a_drawKeyPoints("ui.Draw Key Points", false, true)
	, a_kinFuOn("ui.KinFu On", false, true)
	, a_trackColors("ui.Track Colors", true, true)
	, a_volumeVoxelSize("ui.Volume Voxel Size", 0.01f, 0.005f, 0.02f)
	, a_translateBasisX("ui.Tracker Basis X", 0.f, -10.0f, 10.0f)
	, a_translateBasisZ("ui.Tracker Basis Z", 0.f, -10.0f, 10.0f)
	, a_integrationThreshold("ui.Volume Threshold", 0.15f, 0.05f, 1.0f)
	, a_gpuMem("ui.GPU memory free", 0)
	, a_grabberSpeed("ui.Grabber FPS", "0")
	, a_trackerSpeed("ui.Tracker FPS", "0")
	, a_trackerLagFrames("ui.Tracker Lag Frames", "0")
	, a_saverLagFrames("ui.Saver Lag Frames", "0")
	, a_totalPoints("ui.Total Points", "0")
	, a_keyPoints("ui.Key Points", "0")
	, a_confidenceThreshold("ui.Confidence Threshold", 10.0, 0.0, 24.0)
	, a_icpWeight("ui.ICP Weight", 10.0, 0.0, 100.0)
{
	int width = 1280;
	int height = 980;
	int panel = 205;

	width += panel;

	pangolin::Params windowParams;

	windowParams.Set("SAMPLE_BUFFERS", 0);
	windowParams.Set("SAMPLES", 0);

	pangolin::CreateWindowAndBind("Main", width, height, windowParams);

	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glPixelStorei(GL_PACK_ALIGNMENT, 1);

	pangolin::SetFullscreen(m_showcaseMode);

	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);
	glDepthFunc(GL_LESS);

	s_cam = pangolin::OpenGlRenderState(pangolin::ProjectionMatrix(4, 4, 2, 2, 2, 2, 0.01, 100),
		pangolin::ModelViewLookAt(0, 0, -1, 0, 0, 1, pangolin::AxisY));

	pangolin::Display("cam").SetBounds(0, 1.0f, 0, 1.0f, -640 / 480.0)
		.SetHandler(new pangolin::Handler3D(s_cam));

	pangolin::Display("GrabberImage")
		.SetAspect(640.0f / 480.0f);

	std::vector<std::string> labels;
	labels.push_back(std::string("residual"));
	labels.push_back(std::string("threshold"));
	a_resLog.SetLabels(labels);

	a_resPlot = new pangolin::Plotter(&a_resLog, 0, 300, 0, 0.0005f, 30, 0.5f);
	a_resPlot->Track("$i");

	std::vector<std::string> labels2;
	labels2.push_back(std::string("inliers"));
	labels2.push_back(std::string("threshold"));
	a_inLog.SetLabels(labels2);

	a_inPlot = new pangolin::Plotter(&a_inLog, 0, 300, 0, 40000, 30, 0.5);
	a_inPlot->Track("$i");

	if(!m_showcaseMode)
	{
		pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(panel));
		pangolin::Display("multi").SetBounds(pangolin::Attach::Pix(0), 1 / 4.0f, showcaseMode ? 0 : pangolin::Attach::Pix(180), 1.0)
			.SetLayout(pangolin::LayoutEqualHorizontal)
			.AddDisplay(pangolin::Display("GrabberImage"))
			.AddDisplay(*a_resPlot)
			.AddDisplay(*a_inPlot);
	}

	if(m_showcaseMode)
	{
		pangolin::RegisterKeyPressCallback(' ', pangolin::SetVarFunctor<bool>("ui.Reset", true));
	}

	// Set kernel dir
	std::string shadersFolder = getenv("NUI_LOCATION");
	shadersFolder += "\\Shaders";
	m_pCloudDraw = new NuiPangoCloudShader(shadersFolder);
	m_pTexturedCloudDraw = new NuiPangoTexturedCloudShader(shadersFolder);
	m_pMeshDraw = new NuiPangoMeshShader(shadersFolder);
	m_pTexturedMeshDraw = new NuiPangoTexturedMeshShader(shadersFolder);
}

NuiPangoVis::~NuiPangoVis()
{
	delete a_inPlot;
	delete a_resPlot;
	
	pangolin::FreeImage(rgbImg);

	SafeDelete(m_pCloudDraw);
	SafeDelete(m_pTexturedCloudDraw);
	SafeDelete(m_pMeshDraw);
	SafeDelete(m_pTexturedMeshDraw);
}

void NuiPangoVis::preCall()
{
	glClearColor(0.05f * !m_showcaseMode, 0.05f * !m_showcaseMode, 0.3f * !m_showcaseMode, 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	pangolin::Display("cam").Activate(s_cam);
}

void NuiPangoVis::postCall()
{
	GLint cur_avail_mem_kb = 0;
	glGetIntegerv(GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX, &cur_avail_mem_kb);

	int memFree = cur_avail_mem_kb / 1024;

	a_gpuMem = memFree;

	pangolin::FinishFrame();

	glFinish();
}

void NuiPangoVis::drawFrustum(NuiCLMappableData* pData)
{
	if(!pData)
		return;

	/*Eigen::Matrix3f K = Eigen::Matrix3f::Identity();
	K(0, 0) = m_camParams.m_intrinsics.FocalLengthX;
	K(1, 1) = m_camParams.m_intrinsics.FocalLengthY;
	K(0, 2) = m_camParams.m_intrinsics.cx;
	K(1, 2) = m_camParams.m_intrinsics.cy;

	Eigen::Matrix3f Kinv = K.inverse();

	pangolin::glDrawFrustrum(Kinv,
		int(m_camParams.m_intrinsics.cx) * 2,
		int(m_camParams.m_intrinsics.cy) * 2,
		m_camParams.m_cameraPose.matrix(),
		0.1f);*/
	Eigen::Matrix3f K = Eigen::Matrix3f::Identity();
	K(0, 0) = 2;
	K(1, 1) =2;
	K(0, 2) = 2;
	K(1, 2) = 2;

	Eigen::Matrix3f Kinv = K.inverse();

	pangolin::glDrawFrustrum(Kinv,
		4,
		4,
		pData->GetCameraParams().getTransform(),
		0.1f);
}

void NuiPangoVis::drawBoundingBox(NuiCLMappableData* pData)
{
	if(!pData)
		return;
	const SgVec3f& boxMin = pData->GetBoundingBoxMin();
	const SgVec3f& boxMax = pData->GetBoundingBoxMax();

	GLfloat verts[] = {
		boxMin[0], boxMin[1], boxMin[2],
		boxMax[0], boxMin[1], boxMin[2],
		boxMax[0], boxMax[1], boxMin[2],
		boxMin[0], boxMax[1], boxMin[2],
		boxMin[0], boxMin[1], boxMin[2],
		boxMin[0], boxMin[1], boxMax[2],
		boxMin[0], boxMax[1], boxMax[2],
		boxMin[0], boxMax[1], boxMin[2],
		boxMin[0], boxMax[1], boxMax[2],
		boxMax[0], boxMax[1], boxMax[2],
		boxMax[0], boxMin[1], boxMax[2],
		boxMin[0], boxMin[1], boxMax[2],
		boxMax[0], boxMin[1], boxMax[2],
		boxMax[0], boxMin[1], boxMin[2],
		boxMax[0], boxMax[1], boxMin[2],
		boxMax[0], boxMax[1], boxMax[2]
	};    
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, verts);
	glDrawArrays(GL_LINE_STRIP, 0, 16);
	glDisableClientState(GL_VERTEX_ARRAY);
}

void NuiPangoVis::displayImg(const std::string & id, pangolin::GlTexture* img)
{
	glDisable(GL_DEPTH_TEST);

	pangolin::Display(id).Activate();
	img->RenderToViewport(true);

	glEnable(GL_DEPTH_TEST);
}

bool NuiPangoVis::evaluateColorImage(NuiCLMappableData* pData)
{
	if(!pData)
		return false;

	const NuiColorImage& colorImage = pData->GetColorImage();
	const UINT nColorWidth = colorImage.GetWidth();
	const UINT nColorHeight = colorImage.GetHeight();
	const UINT nColorNum = nColorWidth * nColorHeight;
	BGRQUAD* pColorBuffer = colorImage.GetBuffer();
	if(!pColorBuffer || nColorNum == 0)
		return false;

	if(nColorWidth != rgbTex.width || nColorHeight != rgbTex.height)
		rgbTex.Reinitialise(nColorWidth, nColorHeight);

	if(nColorWidth != rgbImg.w || nColorHeight != rgbImg.h)
	{
		pangolin::FreeImage(rgbImg);
		rgbImg.Alloc(nColorWidth, nColorHeight, pangolin::VideoFormatFromString("RGBA"));
	}

	memcpy(rgbImg.ptr, pColorBuffer, nColorNum * sizeof(BGRQUAD));

	rgbTex.Upload(rgbImg.ptr, GL_BGRA, GL_UNSIGNED_BYTE);

	return true;
}

void NuiPangoVis::draw(NuiCLMappableData* pData)
{
	if(!pData)
		return;

	bool bHasEvaluateColorImage = false;
	if(pData->PointIndices().size())
	{
		if(m_pCloudDraw && pData->ColorStream().size())
		{
			if( m_pCloudDraw->initializeBuffers(pData) )
			{
				m_pCloudDraw->drawPoints(s_cam.GetProjectionModelViewMatrix(), a_drawPointSize);
				m_pCloudDraw->uninitializeBuffers();
			}
		}
		else if(m_pTexturedCloudDraw)
		{
			if( m_pTexturedCloudDraw->initializeBuffers(pData) )
			{
				if( evaluateColorImage(pData)  )
				{
					m_pTexturedCloudDraw->drawPoints(s_cam.GetProjectionModelViewMatrix(), rgbTex.tid, a_drawPointSize);
					bHasEvaluateColorImage = true;
				}
				m_pTexturedCloudDraw->uninitializeBuffers();
			}
		}
	}
	else if(pData->TriangleIndices().size())
	{
		if(m_pMeshDraw && pData->ColorStream().size())
		{
			if( m_pMeshDraw->initializeBuffers(pData) )
			{
				m_pMeshDraw->drawMesh(s_cam.GetProjectionModelViewMatrix());
				m_pMeshDraw->uninitializeBuffers();
			}
		}
		else if(m_pTexturedMeshDraw)
		{
			if( m_pTexturedMeshDraw->initializeBuffers(pData) )
			{
				if( evaluateColorImage(pData)  )
				{
					m_pTexturedMeshDraw->drawMesh(s_cam.GetProjectionModelViewMatrix(), rgbTex.tid);
					bHasEvaluateColorImage = true;
				}
				m_pTexturedMeshDraw->uninitializeBuffers();
			}
		}
	}

	if(!m_showcaseMode)
	{
		if(a_drawFrustum)
		{
			drawFrustum(pData);
		}
		if(a_drawBoundingBox)
		{
			drawBoundingBox(pData);
		}
		if( a_drawColorImage )
		{
			if( !bHasEvaluateColorImage )
			{
				bHasEvaluateColorImage = evaluateColorImage(pData);
			}
			if( bHasEvaluateColorImage  )
			{
				displayImg("GrabberImage", &rgbTex);
			}
		}
	}

}

void NuiPangoVis::updateView(NuiCLMappableData* pData)
{
	if(!pData)
		return;

	pangolin::OpenGlMatrix mv;

	Matrix3frm currRot = pData->GetCameraParams().getRotation().inverse();

	Eigen::Quaternionf currQuat(currRot);
	Vector3f forwardVector(0, 0, 1);
	Vector3f upVector(0, 1, 0);

	Vector3f forward = (currQuat * forwardVector).normalized();
	Vector3f up = (currQuat * upVector).normalized();

	Vector3f eye = - pData->GetCameraParams().getTranslation();

	eye -= forward;

	Vector3f at = eye + forward;

	Vector3f z = (eye - at).normalized();  // Forward
	Vector3f x = up.cross(z).normalized(); // Right
	Vector3f y = z.cross(x);

	Eigen::Matrix4d m;
	m << x(0),  x(1),  x(2),  -(x.dot(eye)),
		y(0),  y(1),  y(2),  -(y.dot(eye)),
		z(0),  z(1),  z(2),  -(z.dot(eye)),
		0,     0,     0,              1;

	memcpy(&mv.m[0], m.data(), sizeof(Eigen::Matrix4d));

	s_cam.SetModelViewMatrix(mv);
}

void NuiPangoVis::render(NuiCLMappableData* pData)
{
	if( a_followPose )
	{
		updateView(pData);
	}
	preCall();
	draw(pData);
	postCall();
}
