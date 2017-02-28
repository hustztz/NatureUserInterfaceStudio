#include "NuiKinfuCPUDepthTracker.h"

#include "NuiKinfuCPUFrame.h"
//#include "NuiKinfuCPUScene.h"
#include "../../NuiKinfuCameraState.h"

#include "Foundation/NuiDebugMacro.h"
#include "Shape/NuiCLMappableData.h"

#include <iostream>
#include <boost/smart_ptr.hpp>

#define KINFU_ICP_CORESPS_NUM 29
#define WORK_GROUP_SIZE 128

#define NAN_FLOAT -99999.0f

bool _IsNan(Vector3f data) { return (NAN_FLOAT == data.x) || (NAN_FLOAT == data.y) ||(NAN_FLOAT == data.z); }

using Eigen::AngleAxisf;

NuiKinfuCPUDepthTracker::NuiKinfuCPUDepthTracker(const NuiTrackerConfig& config, UINT nWidth, UINT nHeight)
	: m_configuration(config)
	, m_nWidth(nWidth)
	, m_nHeight(nHeight)
	, m_error(0.0f)
	, m_count(0.0f)
{
	m_iterations = config.iterations;
	AcquireBuffers();
}

NuiKinfuCPUDepthTracker::~NuiKinfuCPUDepthTracker()
{
	ReleaseBuffers();
}

void	NuiKinfuCPUDepthTracker::AcquireBuffers()
{
	for (UINT i = 0; i < m_iterations.size(); i ++)
	{
		NuiFloatImage* depths = new NuiFloatImage();
		depths->AllocateBuffer(m_nWidth>>i, m_nHeight>>i);
		m_depthsHierarchy.push_back(depths);

		NuiFloat3Image* vertices = new NuiFloat3Image();
		vertices->AllocateBuffer(m_nWidth>>i, m_nHeight>>i);
		m_verticesHierarchy.push_back(vertices);

		NuiFloat3Image* normals = new NuiFloat3Image();
		normals->AllocateBuffer(m_nWidth>>i, m_nHeight>>i);
		m_normalsHierarchy.push_back(normals);
	}

	m_verticesPrev.AllocateBuffer(m_nWidth, m_nHeight);
	m_normalsPrev.AllocateBuffer(m_nWidth, m_nHeight);
}

void	NuiKinfuCPUDepthTracker::ReleaseBuffers()
{
	for (UINT i = 0; i < m_iterations.size(); i ++)
	{
		NuiFloatImage* depths = m_depthsHierarchy.at(i);
		SafeDelete(depths);

		NuiFloat3Image* vertices = m_verticesHierarchy.at(i);
		SafeDelete(vertices);

		NuiFloat3Image* normals = m_normalsHierarchy.at(i);
		SafeDelete(normals);
	}
	m_depthsHierarchy.clear();
	m_verticesHierarchy.clear();
	m_normalsHierarchy.clear();

	m_verticesPrev.Clear();
	m_normalsPrev.Clear();
}

bool NuiKinfuCPUDepthTracker::log(const std::string& fileName) const
{
	return m_configuration.log(fileName);
}

bool NuiKinfuCPUDepthTracker::EvaluateFrame(NuiKinfuFrame* pFrame, NuiKinfuCameraState* pCameraState)
{
	// half sample the input depth maps into the pyramid levels
	if(!pFrame)
		return false;
	NuiKinfuCPUFrame* pCPUFrame = dynamic_cast<NuiKinfuCPUFrame*>(pFrame);
	if(!pCPUFrame)
		return false;
	// filter the input depth map
	SmoothDepths(pCPUFrame->GetDepthsBuffer());
	SubSampleDepths();

	if(!pCameraState)
		return false;

	Depth2vertex(pCameraState->GetCameraPos().getIntrinsics());
	Vertex2Normal();
	pCPUFrame->SetNormalsBuffer(m_normalsHierarchy.at(0)->GetBuffer());

	return true;
}

bool NuiKinfuCPUDepthTracker::EstimatePose(NuiKinfuCameraState* pCameraState, Eigen::Affine3f *hint)
{
	return IterativeClosestPoint(pCameraState, hint);
}

void NuiKinfuCPUDepthTracker::FeedbackPose(NuiKinfuCameraState* pCameraState)
{
	if(!pCameraState)
		return;

	const Matrix3frm& rot = pCameraState->GetCameraPos().getRotation();
	const Vector3f& trans = pCameraState->GetCameraPos().getTranslation();
	// Transform
	NuiFloat3Image* verticesImg = m_verticesHierarchy.at(0);
	NuiFloat3Image* normalsImg = m_normalsHierarchy.at(0);
	if(!normalsImg || !verticesImg)
		return;

	Vector3f* verticesBuffer = verticesImg->GetBuffer();
	Vector3f* normalsBuffer = normalsImg->GetBuffer();
	Vector3f* verticesPrevBuffer = m_verticesPrev.GetBuffer();
	Vector3f* normalsPrevBuffer = m_normalsPrev.GetBuffer();
	if(!normalsBuffer || !verticesBuffer || !verticesPrevBuffer || !normalsPrevBuffer)
		return;

	for (UINT y = 0; y < m_nHeight; y++)
	{
		for (UINT x = 0; x < m_nWidth; x++)
		{
			const UINT id = y * m_nWidth + x;
			Vector3f vert_src = verticesBuffer[id];
			if(_IsNan(vert_src))
			{
				verticesPrevBuffer[id] = Vector3f(NAN_FLOAT, NAN_FLOAT, NAN_FLOAT);
				normalsPrevBuffer[id] = Vector3f(NAN_FLOAT, NAN_FLOAT, NAN_FLOAT);
				continue;
			}
			verticesPrevBuffer[id] = rot * vert_src + trans;
			Vector3f norm_src = normalsBuffer[id];
			if(_IsNan(vert_src))
			{
				normalsPrevBuffer[id] = Vector3f(NAN_FLOAT, NAN_FLOAT, NAN_FLOAT);
				continue;
			}
			normalsPrevBuffer[id] = rot * norm_src;
		}
	}
}

void	NuiKinfuCPUDepthTracker::FeedbackPose(NuiKinfuCameraState* pCameraState, NuiKinfuScene* pScene)
{
	if(!pScene)
		return;
	NuiKinfuOpenCLScene* pCLScene = dynamic_cast<NuiKinfuOpenCLScene*>(pScene);
	if(!pCLScene)
		return;

	if(!pCameraState)
		return;
	NuiKinfuOpenCLCameraState* pCLCamera = dynamic_cast<NuiKinfuOpenCLCameraState*>(pCameraState->GetDeviceCache());
	if(!pCLCamera)
		return;
	const NuiCameraPos& cameraPos = pCameraState->GetCameraPos();
	cl_mem transformCL = pCLCamera->GetCameraTransformBuffer();
	cl_mem cameraParamsCL = pCLCamera->GetCameraParamsBuffer();

	pCLScene->raycastRender(
		m_verticesPrev,
		m_normalsPrev,
		NULL,
		cameraParamsCL,
		transformCL,
		m_nWidth, m_nHeight,
		cameraPos.getSensorDepthMin(), cameraPos.getSensorDepthMax()
		);
}

#define MEAN_SIGMA_L 1.2232f

void NuiKinfuCPUDepthTracker::SmoothDepths(float* floatDepths)
{
	assert(floatDepths);
	if(!floatDepths || m_depthsHierarchy.size() == 0)
		return;

	UINT filterRadius = m_configuration.filter_radius;
	float depthThreshold = m_configuration.depth_threshold;

	NuiFloatImage* depths0Img = m_depthsHierarchy.at(0);
	float* depths0Buffer = depths0Img->GetBuffer();
	for (UINT y = 0; y < m_nHeight; y++)
	{
		for (UINT x = 0; x < m_nWidth; x++)
		{
			const UINT centerId = y * m_nWidth + x;
			float center = floatDepths[centerId];

			depths0Buffer[centerId] = NAN_FLOAT;
			if(center < 0.0f)
			{
				continue;
			}

			float sigma_z = 1.0f / (0.0012f + 0.0019f*(center - 0.4f)*(center - 0.4f) + 0.0001f / sqrt(center) * 0.25f);
			float sumDepth = 0;
			float sumWeight = 0;

			for(UINT cy = -filterRadius; cy <= filterRadius; ++ cy)
			{
				for(UINT cx = -filterRadius; cx <= filterRadius; ++ cx)
				{
					const UINT nearX = x + cx;
					const UINT nearY = y + cy;
					if( nearX>=0 && nearX<m_nWidth && nearY>=0 && nearY<m_nHeight )
					{
						const UINT nearId = nearY * m_nWidth + nearX;
						float near = floatDepths[nearId];
						float diff = fabs(center - near);
						if(near > 0.0f && diff < depthThreshold)
						{
							float depth2 = diff * diff;
							// Different from InfiniTAM
							float weight = expf(-0.5f * ((std::abs(cx) + std::abs(cy))*MEAN_SIGMA_L*MEAN_SIGMA_L + depth2 * sigma_z * sigma_z));

							sumDepth += near * weight;
							sumWeight += weight;
						}
					}
				}
			}
			depths0Buffer[centerId] = sumDepth/sumWeight;
		}
	}
}

void NuiKinfuCPUDepthTracker::SubSampleDepths()
{
	// Sub sample
	float depthThreshold = m_configuration.depth_threshold;
	const UINT subSampleRadius = 1;
	for (UINT i = 1; i < m_iterations.size(); ++i)
	{
		NuiFloatImage* depthsSrcImg = m_depthsHierarchy.at(i-1);
		NuiFloatImage* depthsDstImg = m_depthsHierarchy.at(i);
		if(!depthsSrcImg || !depthsDstImg)
			continue;

		float* depthsSrcBuffer = depthsSrcImg->GetBuffer();
		float* depthsDstBuffer = depthsDstImg->GetBuffer();
		if(!depthsSrcBuffer || !depthsDstBuffer)
			continue;

		UINT rangeX = m_nWidth >> i;
		UINT rangeY = m_nHeight >> i;
		for (UINT y = 0; y < rangeY; y++)
		{
			for (UINT x = 0; x < rangeX; x++)
			{
				const UINT dstId = y * rangeX + x;

				const UINT src_x = x << 1;
				const UINT src_y = y << 1;
				const UINT src_size_x = rangeX << 1;
				const UINT srcId = src_y * src_size_x + src_x;
				float center = depthsSrcBuffer[srcId];

				float sumDepth = 0.0f;
				int sumWeight = 0;
				for(UINT cy = -subSampleRadius; cy <= subSampleRadius; ++ cy)
				{
					for(UINT cx = -subSampleRadius; cx <= subSampleRadius; ++ cx)
					{
						const UINT nearX = x + cx;
						const UINT nearY = y + cy;
						if( nearX>=0 && nearX<m_nWidth && nearY>=0 && nearY<m_nHeight )
						{
							const UINT nearId = nearY * m_nWidth + nearX;
							float near = depthsSrcBuffer[nearId];
							if(near > 0.0f && (center == NAN_FLOAT || fabs(center - near) < depthThreshold))
							{
								sumDepth += near;
								sumWeight += 1;
							}
						}
					}
				}
				depthsDstBuffer[dstId] = (sumWeight > 0) ? sumDepth/sumWeight : -1.0f;
			}
		}
	}
}

void NuiKinfuCPUDepthTracker::Depth2vertex(NuiCameraIntrinsics cameraIntrics)
{
	for (UINT i = 0; i < m_iterations.size(); ++i)
	{
		int div = 1 << i;

		// depth2vertex
		NuiFloatImage* depthsImg = m_depthsHierarchy.at(i);
		NuiFloat3Image* verticesImg = m_verticesHierarchy.at(i);
		if(!depthsImg || !verticesImg)
			continue;

		float* depthsBuffer = depthsImg->GetBuffer();
		Vector3f* verticesBuffer = verticesImg->GetBuffer();
		if(!depthsBuffer || !verticesBuffer)
			continue;

		UINT rangeX = m_nWidth >> i;
		UINT rangeY = m_nHeight >> i;
		for (UINT y = 0; y < rangeY; y++)
		{
			for (UINT x = 0; x < rangeX; x++)
			{
				const UINT id = y * rangeX + x;
				float dp = depthsBuffer[id];

				if(dp > 0.0f)
				{
					const float intr_fx_inv = div / cameraIntrics.m_fx;
					const float intr_fy_inv = div / cameraIntrics.m_fy;
					const float intr_cx = cameraIntrics.m_cx / div;
					const float intr_cy = cameraIntrics.m_cy / div;
					verticesBuffer[id].x = dp * ((float)x - intr_cx) * intr_fx_inv;
					verticesBuffer[id].y = dp * ((float)y - intr_cy) * intr_fy_inv;
					verticesBuffer[id].z = dp;
				}
				else
				{
					verticesBuffer[id] = Vector3f(NAN_FLOAT, NAN_FLOAT, NAN_FLOAT);
				}
			}
		}
	}
}

void	NuiKinfuCPUDepthTracker::Vertex2Normal()
{
	float depthThreshold = m_configuration.depth_threshold;

	for (UINT i = 0; i < m_iterations.size(); ++i)
	{
		NuiFloat3Image* verticesImg = m_verticesHierarchy.at(i);
		NuiFloat3Image* normalsImg = m_normalsHierarchy.at(i);
		if(!normalsImg || !verticesImg)
			continue;

		Vector3f* verticesBuffer = verticesImg->GetBuffer();
		Vector3f* normalsBuffer = normalsImg->GetBuffer();
		if(!normalsBuffer || !verticesBuffer)
			continue;

		UINT rangeX = m_nWidth >> i;
		UINT rangeY = m_nHeight >> i;
		for (UINT y = 0; y < rangeY; y++)
		{
			for (UINT x = 0; x < rangeX; x++)
			{
				const UINT centerId = y * rangeX + x;
				Vector3f center = verticesBuffer[centerId];
				if(!_IsNan(center))
				{
					Vector3f left = center;
					if(x > 0)
					{
						left = verticesBuffer[centerId-1];
						if(_IsNan(left) || fabs(center.z - left.z) > depthThreshold)
							left = center;
					}
					Vector3f right = center;
					if(x < rangeX-1)
					{
						right = verticesBuffer[centerId+1];
						if(_IsNan(right) || fabs(center.z - right.z) > depthThreshold)
							right = center;
					}
					Vector3f up = center;
					if(y > 0)
					{
						up = verticesBuffer[centerId-rangeX];
						if(_IsNan(up) || fabs(center.z - up.z) > depthThreshold)
							up = center;
					}
					Vector3f down = center;
					if(y < rangeY-1)
					{
						down = verticesBuffer[centerId+rangeX];
						if(_IsNan(down) || fabs(center.z - down.z) > depthThreshold)
							down = center;
					}

					// gradients x and y
					Vector3f diff_x = right - left;
					Vector3f diff_y = down - up;

					// cross product
					Vector3f outNormal;
					outNormal.x = (diff_x.y * diff_y.z - diff_x.z*diff_y.y);
					outNormal.y = (diff_x.z * diff_y.x - diff_x.x*diff_y.z);
					outNormal.z = (diff_x.x * diff_y.y - diff_x.y*diff_y.x);
					if(outNormal.x == 0.0f && outNormal.y == 0.0f && outNormal.z == 0.0f)
					{
						normalsBuffer[centerId] = Vector3f(NAN_FLOAT, NAN_FLOAT, NAN_FLOAT);
					}
					float norm = 1.0f / sqrt((outNormal.x * outNormal.x + outNormal.y * outNormal.y + outNormal.z * outNormal.z));
					normalsBuffer[centerId] = outNormal * norm;
				}
				else
				{
					normalsBuffer[centerId] = Vector3f(NAN_FLOAT, NAN_FLOAT, NAN_FLOAT);
				}
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
// Iterative Closest Point
bool NuiKinfuCPUDepthTracker::IterativeClosestPoint(NuiKinfuCameraState* pCameraState, Eigen::Affine3f *hint)
{
	if(!pCameraState)
		return false;
	NuiKinfuOpenCLCameraState* pCLCamera = dynamic_cast<NuiKinfuOpenCLCameraState*>(pCameraState->GetDeviceCache());
	if(!pCLCamera)
		return false;

	cl_mem cameraParamsCL = pCLCamera->GetCameraParamsBuffer();
	if(!cameraParamsCL)
		return false;

	// Get the kernel
	cl_kernel icpKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_ICP_BLOCK);
	assert(icpKernel);
	if (!icpKernel)
	{
		NUI_ERROR("Get kernel 'E_ICP_BLOCK' failed!\n");
		return false;
	}

	cl_kernel sumKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_ICP_SUM);
	assert(sumKernel);
	if (!sumKernel)
	{
		NUI_ERROR("Get kernel 'E_ICP_SUM' failed!\n");
		return false;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();
	cl_uint idx = 0;
	size_t kernelGlobalSize[1] = {m_nWidth * m_nHeight};
	size_t local_ws[1] = {WORK_GROUP_SIZE};
	boost::scoped_array<float> corespResult(new float[kernelGlobalSize[0] * KINFU_ICP_CORESPS_NUM / WORK_GROUP_SIZE]);

	Matrix3frm Rcurr;
	Vector3f tcurr;
	if(hint)
	{
		Rcurr = hint->rotation().matrix();
		tcurr = hint->translation().matrix();
	}
	else
	{
		const NuiCameraPos& cameraPos = pCameraState->GetCameraPos();
		Rcurr = cameraPos.getRotation(); // tranform to global coo for ith camera pose
		tcurr = cameraPos.getTranslation();
	}
	cl_mem previousTransform = pCLCamera->GetCameraTransformBuffer();

	/** \brief array with IPC iteration numbers for each pyramid level */
	int LEVELS = (int)m_iterations.size();

	//ScopeTime time("icp-all");
	for (int level_index = LEVELS-1; level_index>=0; --level_index)
	{
		int div = 1 << level_index;
		int iter_num = m_iterations[level_index];
		for (int iter = 0; iter < iter_num; ++iter)
		{
			idx = 0;
			err = clSetKernelArg(icpKernel, idx++, sizeof(cl_int), &div);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(cl_mem), &cameraParamsCL);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(cl_mem), &m_verticesArrCL[level_index]);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(cl_mem), &m_normalsArrCL[level_index]);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(float)*8, Rcurr.data());
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(float), Rcurr.data()+8);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(float)*4, tcurr.data());
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(cl_mem), &m_verticesPrevArrCL[level_index]);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(cl_mem), &m_normalsPrevArrCL[level_index]);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(cl_mem), &previousTransform);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(float), &m_configuration.dist_threshold);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(float), &m_configuration.normal_threshold);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(icpKernel, idx++, sizeof(cl_mem), &m_corespsBlocksCL);
			NUI_CHECK_CL_ERR(err);

			// Run kernel to calculate
			kernelGlobalSize[0] = (m_nWidth >> level_index) * (m_nHeight >> level_index);
			err = clEnqueueNDRangeKernel(
				queue,
				icpKernel,
				1,
				nullptr,
				kernelGlobalSize,
				local_ws,
				0,
				NULL,
				NULL
			);
			NUI_CHECK_CL_ERR(err);

			UINT size = (UINT)(std::ceil( (float)kernelGlobalSize[0] / (float)local_ws[0] ));

			idx = 0;
			err = clSetKernelArg(sumKernel, idx++, sizeof(cl_mem), &m_corespsBlocksCL);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(sumKernel, idx++, sizeof(cl_mem), &m_corespsCL);
			NUI_CHECK_CL_ERR(err);
			err = clSetKernelArg(sumKernel, idx++, sizeof(cl_uint), &size);
			NUI_CHECK_CL_ERR(err);

			// Run kernel to calculate
			UINT nblocks = (UINT)( std::ceil( (float)size / (float)WORK_GROUP_SIZE) );
			kernelGlobalSize[0] = nblocks * WORK_GROUP_SIZE;
			err = clEnqueueNDRangeKernel(
				queue,
				sumKernel,
				1,
				nullptr,
				kernelGlobalSize,
				local_ws,
				0,
				NULL,
				NULL
				);
			NUI_CHECK_CL_ERR(err);

			err = clEnqueueReadBuffer(
				queue,
				m_corespsCL,
				CL_TRUE,//blocking
				0,
				nblocks * KINFU_ICP_CORESPS_NUM * sizeof(cl_float),
				corespResult.get(),
				0,
				NULL,
				NULL
			);
			NUI_CHECK_CL_ERR(err);

			m_error = 0.0f;
			m_count = 0.0f;
			for(UINT n = 0; n < nblocks; ++n)
			{
				UINT stride = n * KINFU_ICP_CORESPS_NUM;
				m_error += corespResult[stride + KINFU_ICP_CORESPS_NUM-2];
				m_count += corespResult[stride + KINFU_ICP_CORESPS_NUM-1];
			}
#ifdef _DEBUG
			//For debug
			//std::cout << "icpcount:" << icpCount << "\t" << "icperror:" << sqrt(icpError) / icpCount << std::endl;
#endif
			m_error = (m_count > 0.f) ? (sqrt(m_error) / m_count) : std::numeric_limits<float>::max();
			if((m_count < 1.f) || m_error < 1e-5f)
			{
				break;
			}
			/*else
			{
				std::cout << "icpcount:" << icpCount << "\t" << "icperror:" << sqrt(icpError) / icpCount << std::endl;
			}*/

			Eigen::Matrix<double, 6, 6, Eigen::RowMajor> A;
			Eigen::Matrix<double, 6, 1> b;

			int shift = 0;
			for (int i = 0; i < 6; ++i)  //rows
			{
				for (int j = i; j < 7; ++j)    // cols + b
				{
					float value = 0.0f;
					for(UINT n = 0; n < nblocks; ++n)
					{
						UINT stride = n * KINFU_ICP_CORESPS_NUM;
						value +=  corespResult[stride + shift];
					}
					if (j == 6)       // vector b
						b[i] = value;
					else
						A(j,i) = A(i,j) = value;
					shift++;
				}
			}

			//checking nullspace
			double det = A.determinant ();

			if (fabs (det) < 1e-15 || _isnan (det))
			{
				if (_isnan (det)) std::cout << "qnan" << std::endl;

				return (false);
			}
			//float maxc = A.maxCoeff();

			Eigen::Matrix<float, 6, 1> result = A.llt ().solve (b).cast<float>();
			//Eigen::Matrix<float, 6, 1> result = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);

			float alpha = result (0);
			float beta  = result (1);
			float gamma = result (2);

			Eigen::Matrix3f Rinc = (Eigen::Matrix3f)AngleAxisf (gamma, Vector3f::UnitZ ()) * AngleAxisf (beta, Vector3f::UnitY ()) * AngleAxisf (alpha, Vector3f::UnitX ());
			Vector3f tinc = result.tail<3> ();

			//compose
			tcurr = Rinc * tcurr + tinc;
			Rcurr = Rinc * Rcurr;
		}
	}

	pCameraState->UpdateCameraTransform(Rcurr, tcurr);

#ifdef _DEBUG
	//For debug
	//std::cout << "t:" << tcurr[0] << "\t" << tcurr[1] << "\t" << tcurr[2] << std::endl;
#endif

	return true;
}


bool NuiKinfuCPUDepthTracker::VerticesToMappablePosition(NuiCLMappableData* pMappableData)
{
	assert(pMappableData);
	if(!pMappableData)
		return false;

	if(!m_verticesHierarchyCL[0])
		return false;

	const UINT nPointsNum = m_nWidth * m_nHeight;

	pMappableData->SetBoundingBox(SgVec3f(-256.0f / 370.0f, -212.0f / 370.0f, 0.4f),
		SgVec3f((m_nWidth-256.0f) / 370.0f, (m_nHeight-212.0f) / 370.0f, 4.0f));

	NuiMappableAccessor::asVectorImpl(pMappableData->TriangleIndices())->data().clear();
	NuiMappableAccessor::asVectorImpl(pMappableData->WireframeIndices())->data().clear();

	std::vector<unsigned int>& clPointIndices =
		NuiMappableAccessor::asVectorImpl(pMappableData->PointIndices())->data();
	if(clPointIndices.size() != nPointsNum)
	{
		clPointIndices.resize(nPointsNum);
		for (UINT i = 0; i < nPointsNum; ++i)
		{
			clPointIndices[i] = i;
		}
		pMappableData->SetIndexingDirty(true);
	}

	if( nPointsNum != pMappableData->PositionStream().size() )
	{
		NuiMappableAccessor::asVectorImpl(pMappableData->PositionStream())->data().resize(nPointsNum);
	}

	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();
	// 
	err = clFinish(queue);
	NUI_CHECK_CL_ERR(err);

	cl_mem positionsGL = NuiOpenCLBufferFactory::asPosition3fBufferCL(pMappableData->PositionStream());
	// Acquire OpenGL objects before use
	cl_mem glObjs[] = {
		positionsGL
	};

	openclutil::enqueueAcquireHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);

	err = clEnqueueCopyBuffer(
		queue,
		m_verticesHierarchyCL[0],
		positionsGL,
		0,
		0,
		nPointsNum * 3 * sizeof(float),
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	err = clFinish(queue);
	NUI_CHECK_CL_ERR(err);

	// Release OpenGL objects
	openclutil::enqueueReleaseHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);

	pMappableData->SetStreamDirty(true);

	return true;
}

bool	NuiKinfuCPUDepthTracker::BufferToMappableTexture(NuiCLMappableData* pMappableData, BufferType bufferType)
{
	assert(pMappableData);
	if(!pMappableData)
		return false;

	if(!m_verticesHierarchyCL[0])
		return false;

	// Get the kernel
	cl_kernel rgbaKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_FLOAT3_TO_TEXTURE);
	assert(rgbaKernel);
	if (!rgbaKernel)
	{
		NUI_ERROR("Get kernel 'E_FLOAT3_TO_RGBA' failed!\n");
		return false;
	}

	if( m_nWidth != pMappableData->ColorTex().width() || m_nHeight != pMappableData->ColorTex().height())
	{
		NuiTextureMappableAccessor::updateImpl(
			pMappableData->ColorTex(),
			m_nWidth,
			m_nHeight,
			NULL
			);
	}
	cl_mem texGL = NuiOpenCLBufferFactory::asTexture2DCL(pMappableData->ColorTex());

	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();
	// 
	err = clFinish(queue);
	NUI_CHECK_CL_ERR(err);

	// Acquire OpenGL objects before use
	cl_mem glObjs[] = {
		texGL
	};

	openclutil::enqueueAcquireHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(rgbaKernel, idx++, sizeof(cl_mem), &m_verticesHierarchyCL[0]);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(rgbaKernel, idx++, sizeof(cl_mem), &texGL);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	size_t kernelGlobalSize[2] = { m_nWidth, m_nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		rgbaKernel,
		2,
		nullptr,
		kernelGlobalSize,
		nullptr,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	err = clFinish(queue);
	NUI_CHECK_CL_ERR(err);

	// Release OpenGL objects
	openclutil::enqueueReleaseHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, nullptr);

	return true;
}

