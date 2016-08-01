#pragma once

#include "NuiICPConfig.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;
typedef Eigen::Vector3f Vector3f;

class NuiKinfuTransform;

class NuiPyramidICP
{
public:
	NuiPyramidICP(const NuiICPConfig& config, UINT nWidth, UINT nHeight);
	~NuiPyramidICP();

	bool	log(const std::string& fileName) const;

	void	input(cl_mem floatDepthsCL, cl_mem cameraParamsCL);
	bool	run(cl_mem cameraParamsCL, NuiKinfuTransform* pTransform, Eigen::Affine3f *hint);
	void	transformPrevs(cl_mem transformCL);
	void	resizePrevs();
	void	copyPrevs();

	float	getError() const { return m_error; }
	float	getCount() const { return m_count; }
	
	cl_mem	getColorsCL() const { return m_colorsArrCL[0]; }
	cl_mem	getNormalsCL() const { return m_normalsArrCL[0]; }
	cl_mem	getPrevVerticesCL() const { return m_verticesPrevArrCL[0]; }
	cl_mem	getPrevNormalsCL() const { return m_normalsPrevArrCL[0]; }
	cl_mem	getPrevColorsCL() const { return m_colorsPrevArrCL[0]; }

protected:
	void	AcquireBuffers(bool bHasColor);
	void	ReleaseBuffers();

	void	GenerateGaussianBuffer();
	void	SmoothDepths(cl_mem floatDepthsCL);
	void	PyrDown();
	void	NormalEst(cl_mem cameraParamsCL);
	bool	IterativeClosestPoint(cl_mem cameraParamsCL, NuiKinfuTransform* pTransform, Eigen::Affine3f *hint);
	//bool	ColorIterativeClosestPoint(NuiCameraPos* pPos, Eigen::Affine3f *hint);
	void    ResizePrevMaps();
	void	TransformPrevMaps(cl_mem transformCL);
	void    CopyPrevMaps();
	void	CopyPrevColorMaps();

private:
	cl_mem m_gaussianCL;
	typedef std::vector<cl_mem> GPUBuffers;
	GPUBuffers m_depthsArrCL;
	GPUBuffers m_verticesArrCL;
	GPUBuffers m_normalsArrCL;
	GPUBuffers m_colorsArrCL;
	GPUBuffers m_verticesPrevArrCL;
	GPUBuffers m_normalsPrevArrCL;
	GPUBuffers m_colorsPrevArrCL;
	cl_mem m_corespsBlocksCL;
	cl_mem m_corespsCL;

	NuiICPConfig m_configuration;
	std::vector<UINT> m_iterations;
	UINT m_nWidth, m_nHeight;

	float m_error;
	float m_count;
};