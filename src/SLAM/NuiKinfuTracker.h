#pragma once

#include "NuiICPConfig.h"
#include "NuiKinfuTransform.h"
#include "Shape\NuiCameraPos.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"
#include "Shape\NuiImageBuffer.h"

//Forwards
class NuiCLMappableData;
class NuiKinfuVolume;
class NuiPyramidICP;
struct NuiCLCameraParams;

typedef Eigen::Vector3i Vector3i;

class NuiKinfuTracker
{
public:
	NuiKinfuTracker(NuiICPConfig& icpConfig, UINT nWidth, UINT nHeight, UINT nColorWidth, UINT nColorHeight);
	NuiKinfuTracker();
	~NuiKinfuTracker();

	bool	isInit() const { return (m_icp ? true : false); }
	void	initialize(NuiICPConfig& icpConfig, UINT nWidth, UINT nHeight, UINT nColorWidth, UINT nColorHeight);

	/** \brief Performs the tracker reset to initial  state. It's used if case of camera tracking fail.  */
	void	reset(const Vector3f& translateBasis);

	bool	RunTracking(UINT16* pDepths,
		UINT nPointNum,
		UINT16 minDepth,
		UINT16 maxDepth,
		ColorSpacePoint* pDepthToColor,
		const NuiColorImage& image,
		NuiKinfuVolume*	pVolume,
		float intri_fx,
		float intri_fy,
		float intri_cx,
		float intri_cy);

	/** \brief Returns camera pose at given time, default the last pose
        * \param[in] time Index of frame for which camera pose is returned.
        * \return camera pose
        */
    const NuiCameraPos&		getCameraPose (int time = -1) const;
	float					getIcpError() const;
	float					getIcpCount() const;

	bool					previousBufferToData(NuiCLMappableData* pCLData);
	bool					previousNormalImageToData(NuiCLMappableData* pCLData);

protected:
	void	AcquireBuffers(bool bHasColor);
	void	ReleaseBuffers();
	bool	AcquireGLBuffer(NuiCLMappableData* pCLData);
	void	ReleaseGLBuffer();

	void	PassingDepths(float nearPlane, float farPlane);
	void	WriteDepths(UINT16* pDepths, UINT nPositionsNum, UINT16 minDepth, UINT16 maxDepth);
	void	WriteCameraParams(const NuiCLCameraParams& camIntri);
	void	WriteColors(ColorSpacePoint* pDepthToColor, const NuiColorImage& image, UINT nPointsNum);

private:
	NuiPyramidICP*	m_icp;
	NuiKinfuTransform m_transform;

	cl_mem m_positionsGL;
	cl_mem m_rawDepthsCL;
	cl_mem m_floatDepthsCL;
	cl_mem m_colorUVsCL;
	cl_mem m_colorImageCL;
	cl_mem m_colorsCL;
	cl_mem m_cameraParamsCL;
	cl_mem m_outputColorImageCL;

	UINT m_nWidth, m_nHeight;
	UINT m_nColorWidth, m_nColorHeight;

	std::vector<NuiCameraPos> m_frames;
	NuiCameraPos			m_initialPos;
};