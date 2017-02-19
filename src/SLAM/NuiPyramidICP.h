#pragma once

#include "NuiTrackerConfig.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;
typedef Eigen::Vector3f Vector3f;

class NuiKinfuTransform;

class NuiPyramidICP
{
public:
	NuiPyramidICP(const NuiTrackerConfig& config, UINT nWidth, UINT nHeight);
	~NuiPyramidICP();

	bool	log(const std::string& fileName) const;

	void	input(cl_mem floatDepthsCL, cl_mem colorsCL, cl_mem cameraParamsCL);
	bool	run(cl_mem cameraParamsCL, NuiKinfuTransform* pTransform, Eigen::Affine3f *hint);
	void	transformPrevs(cl_mem transformCL);
	void	resizePrevs();
	void	copyPrevs();

	float	getError() const { return m_error; }
	float	getCount() const { return m_count; }
	
	cl_mem	getIntensitiesCL() const { return m_intensitiesArrCL[0]; }
	cl_mem	getNormalsCL() const { return m_normalsArrCL[0]; }
	cl_mem	getPrevVerticesCL() const { return m_verticesPrevArrCL[0]; }
	cl_mem	getPrevNormalsCL() const { return m_normalsPrevArrCL[0]; }
	cl_mem	getPrevIntensitiesCL() const { return m_intensitiesPrevArrCL[0]; }

protected:
	void	AcquireBuffers(bool bHasIntensity);
	void	ReleaseBuffers();

	void	GenerateGaussianBuffer();
	void	SmoothDepths(cl_mem floatDepthsCL);
	void	ColorsToIntensity(cl_mem colorsCL);
	void	NormalEst(cl_mem cameraParamsCL);
	bool	IterativeClosestPoint(cl_mem cameraParamsCL, NuiKinfuTransform* pTransform, Eigen::Affine3f *hint);
	bool	IntensityIterativeClosestPoint(cl_mem cameraParamsCL, NuiKinfuTransform* pTransform, Eigen::Affine3f *hint);
	void	CopyPrevIntensityMaps();

private:
	cl_mem m_gaussianCL;
	typedef std::vector<cl_mem> GPUBuffers;
	GPUBuffers m_depthsArrCL;
	GPUBuffers m_verticesArrCL;
	GPUBuffers m_normalsArrCL;
	GPUBuffers m_intensitiesArrCL;
	GPUBuffers m_verticesPrevArrCL;
	GPUBuffers m_normalsPrevArrCL;
	GPUBuffers m_intensitiesPrevArrCL;
	GPUBuffers m_intensityDerivsPrevArrCL;
	cl_mem m_corespsBlocksCL;
	cl_mem m_corespsCL;

	NuiTrackerConfig m_configuration;
	std::vector<UINT> m_iterations;
	UINT m_nWidth, m_nHeight;

	float m_error;
	float m_count;
};