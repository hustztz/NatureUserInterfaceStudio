#include "NuiKinfuCPUDepthTracker.h"

#include "NuiKinfuCPUFrame.h"
//#include "NuiKinfuCPUScene.h"
#include "../../NuiKinfuCameraState.h"

#include "Foundation/NuiDebugMacro.h"
#include "Foundation/NuiCholesky.h"
#include "Shape/NuiCLMappableData.h"

#include <iostream>
#include <boost/smart_ptr.hpp>

#define KINFU_ICP_CORESPS_NUM 29
#define WORK_GROUP_SIZE 128

#define NAN_FLOAT -std::numeric_limits<float>::max()

static bool _IsNan(Vector3f data) { return ((NAN_FLOAT == data[0]) || (NAN_FLOAT == data[1]) || (NAN_FLOAT == data[2])); }

using Eigen::AngleAxisf;

NuiKinfuCPUDepthTracker::NuiKinfuCPUDepthTracker(const NuiTrackerConfig& config, UINT nWidth, UINT nHeight)
	: m_configuration(config)
	, m_nWidth(nWidth)
	, m_nHeight(nHeight)
	, m_error(0.0f)
	, m_numValidPoints(0)
{
	AcquireBuffers();
}

NuiKinfuCPUDepthTracker::~NuiKinfuCPUDepthTracker()
{
	ReleaseBuffers();
}

void	NuiKinfuCPUDepthTracker::AcquireBuffers()
{
	const NuiTrackerConfig::ITERATION_CLASS& iterations = m_configuration.iterations;
	for (UINT i = 0; i < iterations.size(); i ++)
	{
		NuiFloatImage* depths = new NuiFloatImage();
		depths->AllocateBuffer(m_nWidth>>i, m_nHeight>>i);
		m_depthsHierarchy.push_back(depths);

		NuiFloat3Image* vertices = new NuiFloat3Image();
		vertices->AllocateBuffer(m_nWidth>>i, m_nHeight>>i);
		m_verticesHierarchy.push_back(vertices);
	}
	m_normals.AllocateBuffer(m_nWidth, m_nHeight);
	m_verticesPrev.AllocateBuffer(m_nWidth, m_nHeight);
	m_normalsPrev.AllocateBuffer(m_nWidth, m_nHeight);
}

void	NuiKinfuCPUDepthTracker::ReleaseBuffers()
{
	const NuiTrackerConfig::ITERATION_CLASS& iterations = m_configuration.iterations;
	for (UINT i = 0; i < iterations.size(); i ++)
	{
		NuiFloatImage* depths = m_depthsHierarchy.at(i);
		SafeDelete(depths);

		NuiFloat3Image* vertices = m_verticesHierarchy.at(i);
		SafeDelete(vertices);
	}
	m_depthsHierarchy.clear();
	m_verticesHierarchy.clear();

	m_normals.Clear();
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
	pCPUFrame->SetNormalsBuffer(m_normals.GetBuffer());

	return true;
}

bool NuiKinfuCPUDepthTracker::EstimatePose(NuiKinfuCameraState* pCameraState, Eigen::Affine3f *hint)
{
	return IterativeClosestPoint(pCameraState, hint);
}

void NuiKinfuCPUDepthTracker::TransformBuffers(NuiKinfuCameraState* pCameraState)
{
	if(!pCameraState)
		return;

	const Matrix3frm& rot = pCameraState->GetCameraPos().getRotation();
	const Vector3f& trans = pCameraState->GetCameraPos().getTranslation();
	// Transform
	NuiFloat3Image* verticesImg = m_verticesHierarchy.at(0);
	if(!verticesImg)
		return;

	Vector3f* verticesBuffer = verticesImg->GetBuffer();
	Vector3f* normalsBuffer = m_normals.GetBuffer();
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
	if(!pCameraState)
		return;


	if(pScene)
	{

	}
	else
	{
		TransformBuffers(pCameraState);
	}
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
	for (int y = 0; y < (int)m_nHeight; y++)
	{
		for (int x = 0; x < (int)m_nWidth; x++)
		{
			const int centerId = y * (int)m_nWidth + x;
			float center = floatDepths[centerId];

			depths0Buffer[centerId] = NAN_FLOAT;
			if(center < 0.0f)
			{
				continue;
			}

			float sigma_z = 1.0f / (0.0012f + 0.0019f*(center - 0.4f)*(center - 0.4f) + 0.0001f / sqrt(center) * 0.25f);
			float sumDepth = 0;
			float sumWeight = 0;

			for(int cy = -filterRadius; cy <= filterRadius; ++ cy)
			{
				for(int cx = -filterRadius; cx <= filterRadius; ++ cx)
				{
					const int nearX = x + cx;
					const int nearY = y + cy;
					if( nearX>=0 && nearX<m_nWidth && nearY>=0 && nearY<m_nHeight )
					{
						const int nearId = nearY * m_nWidth + nearX;
						float near = floatDepths[nearId];
						float diff = fabs(center - near);
						if(near > 0.0f && diff < depthThreshold)
						{
							float depth2 = diff * diff;
							// Different from InfiniTAM
							float weight = expf(-0.5f * ((abs(cx) + abs(cy))*MEAN_SIGMA_L*MEAN_SIGMA_L + depth2 * sigma_z * sigma_z));

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
	const NuiTrackerConfig::ITERATION_CLASS& iterations = m_configuration.iterations;
	for (UINT i = 1; i < iterations.size(); ++i)
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
				for(int cy = -subSampleRadius; cy <= subSampleRadius; ++ cy)
				{
					for(int cx = -subSampleRadius; cx <= subSampleRadius; ++ cx)
					{
						const int nearX = x + cx;
						const int nearY = y + cy;
						if( nearX>=0 && nearX<m_nWidth && nearY>=0 && nearY<m_nHeight )
						{
							const int nearId = nearY * m_nWidth + nearX;
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
	const NuiTrackerConfig::ITERATION_CLASS& iterations = m_configuration.iterations;
	for (UINT i = 0; i < iterations.size(); ++i)
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
					verticesBuffer[id][0] = dp * ((float)x - intr_cx) * intr_fx_inv;
					verticesBuffer[id][1] = dp * ((float)y - intr_cy) * intr_fy_inv;
					verticesBuffer[id][2] = dp;
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

	NuiFloat3Image* verticesImg = m_verticesHierarchy.at(0);
	if(!verticesImg)
		return;

	Vector3f* verticesBuffer = verticesImg->GetBuffer();
	Vector3f* normalsBuffer = m_normals.GetBuffer();
	if(!normalsBuffer || !verticesBuffer)
		return;

	for (UINT y = 0; y < m_nHeight; y++)
	{
		for (UINT x = 0; x < m_nWidth; x++)
		{
			const UINT centerId = y * m_nWidth + x;
			Vector3f center = verticesBuffer[centerId];
			if(!_IsNan(center))
			{
				Vector3f left = center;
				if(x > 0)
				{
					left = verticesBuffer[centerId-1];
					if(_IsNan(left) || fabs(center[2] - left[2]) > depthThreshold)
						left = center;
				}
				Vector3f right = center;
				if(x < m_nWidth-1)
				{
					right = verticesBuffer[centerId+1];
					if(_IsNan(right) || fabs(center[2] - right[2]) > depthThreshold)
						right = center;
				}
				Vector3f up = center;
				if(y > 0)
				{
					up = verticesBuffer[centerId-m_nWidth];
					if(_IsNan(up) || fabs(center[2] - up[2]) > depthThreshold)
						up = center;
				}
				Vector3f down = center;
				if(y < m_nHeight-1)
				{
					down = verticesBuffer[centerId+m_nWidth];
					if(_IsNan(down) || fabs(center[2] - down[2]) > depthThreshold)
						down = center;
				}

				// gradients x and y
				Vector3f diff_x = right - left;
				Vector3f diff_y = down - up;

				// cross product
				Vector3f outNormal = diff_x.cross(diff_y);
				/*outNormal.x = (diff_x.y * diff_y.z - diff_x.z*diff_y.y);
				outNormal.y = (diff_x.z * diff_y.x - diff_x.x*diff_y.z);
				outNormal.z = (diff_x.x * diff_y.y - diff_x.y*diff_y.x);*/
				if(outNormal[0] == 0.0f && outNormal[1] == 0.0f && outNormal[2] == 0.0f)
				{
					normalsBuffer[centerId] = Vector3f(NAN_FLOAT, NAN_FLOAT, NAN_FLOAT);
				}
				//float norm = 1.0f / sqrt((outNormal.x * outNormal.x + outNormal.y * outNormal.y + outNormal.z * outNormal.z));
				normalsBuffer[centerId] = outNormal.normalized();
			}
			else
			{
				normalsBuffer[centerId] = Vector3f(NAN_FLOAT, NAN_FLOAT, NAN_FLOAT);
			}
		}
	}
}

Vector3f NuiKinfuCPUDepthTracker::InterpolateBilinear_withHoles(const Vector3f* source, Vector2f position, UINT nWidth)
{
	Vector3f a, b, c, d;
	Vector3f result;
	Eigen::Vector2i p; Vector2f delta;

	p[0] = (int)floor(position[0]); p[1] = (int)floor(position[1]);
	delta[0] = position[0] - (float)p[0]; delta[1] = position[1] - (float)p[1];

	a = source[p[0] + p[1] * nWidth];
	b = source[(p[0] + 1) + p[1] * nWidth];
	c = source[p[0] + (p[1] + 1) * nWidth];
	d = source[(p[0] + 1) + (p[1] + 1) * nWidth];

	if (a[0] == NAN_FLOAT || a[1] == NAN_FLOAT || a[2] == NAN_FLOAT ||
		b[0] == NAN_FLOAT || b[1] == NAN_FLOAT || b[2] == NAN_FLOAT ||
		c[0] == NAN_FLOAT || c[1] == NAN_FLOAT || c[2] == NAN_FLOAT ||
		d[0] == NAN_FLOAT || d[1] == NAN_FLOAT || d[2] == NAN_FLOAT)
	{
		result[0] = NAN_FLOAT; result[1] = NAN_FLOAT; result[2] = NAN_FLOAT;
		return result;
	}

	result[0] = ((float)a[0] * (1.0f - delta[0]) * (1.0f - delta[1]) + (float)b[0] * delta[0] * (1.0f - delta[1]) +
		(float)c[0] * (1.0f - delta[0]) * delta[1] + (float)d[0] * delta[0] * delta[1]);
	result[1] = ((float)a[1] * (1.0f - delta[0]) * (1.0f - delta[1]) + (float)b[1] * delta[0] * (1.0f - delta[1]) +
		(float)c[1] * (1.0f - delta[0]) * delta[1] + (float)d[1] * delta[0] * delta[1]);
	result[2] = ((float)a[2] * (1.0f - delta[0]) * (1.0f - delta[1]) + (float)b[2] * delta[0] * (1.0f - delta[1]) +
		(float)c[2] * (1.0f - delta[0]) * delta[1] + (float)d[2] * delta[0] * delta[1]);

	return result;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Iterative Closest Point
bool NuiKinfuCPUDepthTracker::IterativeClosestPoint(NuiKinfuCameraState* pCameraState, Eigen::Affine3f *hint)
{
	if(!pCameraState)
		return false;

	const NuiCameraIntrinsics& cameraIntrinsics = pCameraState->GetCameraPos().getIntrinsics();
	const Matrix3frm& Rprev = pCameraState->GetCameraPos().getRotation();
	const Vector3f& tprev = pCameraState->GetCameraPos().getTranslation();

	Matrix3frm Rinv;
	Vector3f tcurr;
	if(hint)
	{
		Rinv = hint->rotation().inverse();
		tcurr = hint->translation().matrix();
	}
	else
	{
		Rinv = Rprev.inverse(); // tranform to global coo for ith camera pose
		tcurr = tprev;
	}

	/** \brief array with IPC iteration numbers for each pyramid level */
	const NuiTrackerConfig::ITERATION_CLASS& iterations = m_configuration.iterations;
	int LEVELS = (int)iterations.size();

	//ScopeTime time("icp-all");
	for (int level_index = LEVELS-1; level_index>=0; --level_index)
	{
		NuiFloat3Image* verticesImg = m_verticesHierarchy.at(level_index);
		if(!verticesImg)
			continue;
		Vector3f* verticesBuffer = verticesImg->GetBuffer();
		if(!verticesBuffer)
			continue;

		const UINT rangeX = m_nWidth >> level_index;
		const UINT rangeY = m_nHeight >> level_index;

		//Vector3f* normalsBuffer = m_normals.GetBuffer();

		int iter_num = iterations[level_index].m_num;
		NuiTrackerConfig::TrackerIterationType iter_type = iterations[level_index].m_type;
		bool bShortIteration = (NuiTrackerConfig::eTracker_Iteration_Both != iter_type);
		const UINT numPara = bShortIteration ? 3 : 6;
		const UINT numParaSQ = bShortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

		// Reset params
		m_error = 1e20f;
		m_numValidPoints = 0;
		float lambda = 1.0;
		Matrix3frm lastGoodRotInv = Rinv;
		Vector3f lastGoodTrans = tcurr;

		for (int iter = 0; iter < iter_num; ++iter)
		{
			float sumF = 0.0f;
			float sumHessian[6 * 6], sumNabla[6];
			memset(sumHessian, 0, sizeof(float) * numParaSQ);
			memset(sumNabla, 0, sizeof(float) * numPara);
			for (UINT y = 0; y < rangeY; y ++)
			{
				for (UINT x = 0; x < rangeX; x ++)
				{
					const UINT id = y * rangeX + x;
					Vector3f vert = verticesBuffer[id];
					if(NAN_FLOAT == vert[0] || NAN_FLOAT == vert[1] || NAN_FLOAT == vert[2])
						continue;

					Vector3f projectedVert = Rinv * (vert - tcurr);
					Vector3f projectedPos = Rprev * projectedVert + tprev;
					Vector2f projPixel( projectedPos[0] * cameraIntrinsics.m_fx / projectedPos[2] + cameraIntrinsics.m_cx,
										projectedPos[1] * cameraIntrinsics.m_fy / projectedPos[2] + cameraIntrinsics.m_cy);
					if(projPixel[0] < 0 || projPixel[0] >= m_nWidth || projPixel[1] < 0 || projPixel[1] >= m_nHeight)
						continue;

					Vector3f referenceVert = InterpolateBilinear_withHoles(m_verticesPrev.GetBuffer(), projPixel, m_verticesPrev.GetWidth());
					if(NAN_FLOAT == referenceVert[0] || NAN_FLOAT == referenceVert[1] || NAN_FLOAT == referenceVert[2])
						continue;

					Vector3f ptDiff = referenceVert - projectedVert;
					float dist = ptDiff[0] * ptDiff[0] + ptDiff[1] * ptDiff[1] + ptDiff[2] * ptDiff[2];
					if (dist > m_configuration.dist_threshold)
						continue;

					Vector3f referenceNorm = InterpolateBilinear_withHoles(m_normalsPrev.GetBuffer(), projPixel, m_normalsPrev.GetWidth());
					if(NAN_FLOAT == referenceNorm[0] || NAN_FLOAT == referenceNorm[1] || NAN_FLOAT == referenceNorm[2])
						continue;

					/*if (normalsBuffer)
					{
						Vector3f norm = normalsBuffer[id];
						float sine = norm.cross(referenceNorm).norm();
						if(sine < m_configuration.normal_threshold)
							continue;
					}*/

					float b = referenceNorm.dot(ptDiff);

					Vector3f row0 = projectedVert.cross(referenceNorm);
					float A[6];
					if (NuiTrackerConfig::eTracker_Iteration_Rotation == iter_type)
					{
						A[0] = row0[0];
						A[1] = row0[1];
						A[2] = row0[2];
					}
					else if (NuiTrackerConfig::eTracker_Iteration_Translation == iter_type)
					{
						A[0] = referenceNorm[0];
						A[1] = referenceNorm[1];
						A[2] = referenceNorm[2];
					}
					else
					{
						A[0] = row0[0];
						A[1] = row0[1];
						A[2] = row0[2];
						A[3] = referenceNorm[0];
						A[4] = referenceNorm[1];
						A[5] = referenceNorm[2];
					}

					float localF = b * b;
					float localHessian[6 + 5 + 4 + 3 + 2 + 1], localNabla[6];
					for (int r = 0, counter = 0; r < numPara; r++)
					{
						localNabla[r] = b * A[r];
						for (int c = 0; c <= r; c++, counter++)
							localHessian[counter] = A[r] * A[c];
					}

					// sum
					m_numValidPoints ++;
					sumF += localF;
					for (int i = 0; i < numPara; i++)
						sumNabla[i] += localNabla[i];
					for (int i = 0; i < numParaSQ; i++)
						sumHessian[i] += localHessian[i];
				}
			}

			// build hessian
			float hessian[6 * 6];
			for (int r = 0, counter = 0; r < numPara; r++)
				for (int c = 0; c <= r; c++, counter++)
					hessian[r + c * 6] = sumHessian[counter];
			for (int r = 0; r < numPara; ++r)
				for (int c = r + 1; c < numPara; c++)
					hessian[r + c * 6] = hessian[c + r * 6];
			float nabla[6];
			memcpy(nabla, sumNabla, numPara * sizeof(float));
			float current_error = (m_numValidPoints > 100) ? sqrt(sumF) / m_numValidPoints : 1e5f;
			if ((m_numValidPoints <= 0) || (current_error > m_error))
			{
				Rinv = lastGoodRotInv;
				tcurr = lastGoodTrans;
				lambda *= 10.0f;
			}
			else
			{
				lastGoodRotInv = Rinv;
				lastGoodTrans = tcurr;
				m_error = current_error;

				for (int i = 0; i < 6*6; ++i) hessian[i] = hessian[i] / m_numValidPoints;
				for (int i = 0; i < 6; ++i) nabla[i] = nabla[i] / m_numValidPoints;
				lambda /= 10.0f;
			}
			for (int i = 0; i < 6; ++i)
				hessian[i+i*6] *= 1.0f + lambda;

			// ComputeDelta
			float step[6];
			for (int i = 0; i < 6; i++)
				step[i] = 0;
			if (bShortIteration)
			{
				float smallHessian[3 * 3];
				for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) smallHessian[r + c * 3] = hessian[r + c * 6];

				NuiCholesky cholA(smallHessian, 3);
				cholA.Backsub(step, nabla);
			}
			else
			{
				NuiCholesky cholA(hessian, 6);
				cholA.Backsub(step, nabla);
			}

			// ApplyDelta
			switch (iter_type)
			{
			case NuiTrackerConfig::eTracker_Iteration_Rotation:
				{
					Eigen::Matrix3f Rinc;
					Rinc.setIdentity();
					Rinc(0,1) = - step[2];
					Rinc(1,0) = step[2];
					Rinc(0,2) = step[1];
					Rinc(2,0) = - step[1];
					Rinc(1,2) = - step[0];
					Rinc(2,1) = step[0];
					Rinv = Rinc * Rinv;
					break;
				}
			case NuiTrackerConfig::eTracker_Iteration_Translation:
				{
					tcurr += Vector3f(step[0], step[1], step[2]);
					break;
				}
			default:
			case NuiTrackerConfig::eTracker_Iteration_Both:
				{
					Eigen::Matrix3f Rinc;
					Rinc.setIdentity();
					Rinc(0,1) = - step[2];
					Rinc(1,0) = step[2];
					Rinc(0,2) = step[1];
					Rinc(2,0) = - step[1];
					Rinc(1,2) = - step[0];
					Rinc(2,1) = step[0];
					Vector3f tinc(step[3], step[4], step[5]);
					Rinv = Rinc * Rinv;
					tcurr = Rinc * tcurr + tinc;
					break;
				}
			}

			// HasConverged
			float stepLength = 0.0f;
			for (int i = 0; i < 6; i++)
				stepLength += step[i] * step[i];

			if (sqrt(stepLength) / 6 < m_configuration.track_threshold)
				break;
		}
	}

	pCameraState->UpdateCameraTransform(Rinv.inverse(), tcurr);

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

	if(!m_verticesHierarchy.at(0))
		return false;

	Vector3f* verticesBuffer = m_verticesHierarchy.at(0)->GetBuffer();
	if(!verticesBuffer)
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

	std::vector<SgVec3f>& clVertices =
		NuiMappableAccessor::asVectorImpl(pMappableData->PositionStream())->data();
	if( nPointsNum != clVertices.size() )
	{
		clVertices.resize(nPointsNum);
	}

	for (UINT i = 0; i < nPointsNum; ++i)
	{
		clVertices[i][0] = verticesBuffer[i][0];
		clVertices[i][1] = verticesBuffer[i][1];
		clVertices[i][2] = verticesBuffer[i][2];
	}
	pMappableData->SetStreamDirty(true);

	return true;
}

bool	NuiKinfuCPUDepthTracker::BufferToMappableTexture(NuiCLMappableData* pMappableData, TrackerBufferType bufferType)
{
	assert(pMappableData);
	if(!pMappableData)
		return false;

	Vector3f* buffer = NULL;
	switch (bufferType)
	{
	case NuiKinfuTracker::eTracker_Vertices:
		buffer = m_verticesHierarchy.at(0) ? m_verticesHierarchy.at(0)->GetBuffer() : NULL;
		break;
	case NuiKinfuTracker::eTracker_Normals:
		buffer = m_normals.GetBuffer();
		break;
	default:
		break;
	}

	if(!buffer)
		return false;

	NuiTextureMappableAccessor::updateImpl(
		pMappableData->ColorTex(),
		m_nWidth,
		m_nHeight,
		buffer
		);

	return true;
}

