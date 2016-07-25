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
	NuiPyramidICP(NuiICPConfig& config, UINT nWidth, UINT nHeight);
	~NuiPyramidICP();

	void	input(cl_mem floatDepthsCL, cl_mem cameraParamsCL);
	bool	run(cl_mem cameraParamsCL, NuiKinfuTransform* pTransform, Eigen::Affine3f *hint);
	void	transformPrevs(cl_mem transformCL);
	void	resizePrevs();
	void	copyPrevs();

	float	getError() const { return m_error; }
	float	getCount() const { return m_count; }
	
	cl_mem	getNormals() const { return m_normalsArrCL[0]; }
	cl_mem	getPrevVertices() const { return m_verticesPrevArrCL[0]; }
	cl_mem	getPrevNormals() const { return m_normalsPrevArrCL[0]; }

protected:
	void	AcquireBuffers();
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

private:
	cl_mem m_gaussianCL;
	typedef std::vector<cl_mem> GPUBuffers;
	GPUBuffers m_depthsArrCL;
	GPUBuffers m_verticesArrCL;
	GPUBuffers m_normalsArrCL;
	GPUBuffers m_verticesPrevArrCL;
	GPUBuffers m_normalsPrevArrCL;
	cl_mem m_corespsBlocksCL;
	cl_mem m_corespsCL;

	NuiICPConfig& m_configuration;
	std::vector<UINT> m_iterations;
	UINT m_nWidth, m_nHeight;

	float m_error;
	float m_count;
};