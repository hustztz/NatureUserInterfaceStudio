#include "NuiKinfuCPUDepthTracker.h"

#include "NuiKinfuCPUUtilities.h"
#include "NuiKinfuCPUFrame.h"
#include "NuiKinfuCPUFeedbackFrame.h"
#include "../NuiKinfuCameraState.h"

#include "Foundation/NuiDebugMacro.h"
#include "Foundation/NuiCholesky.h"

#include <iostream>
#include <boost/smart_ptr.hpp>

#define KINFU_ICP_CORESPS_NUM 29
#define WORK_GROUP_SIZE 128

using Eigen::AngleAxisf;

NuiKinfuCPUDepthTracker::NuiKinfuCPUDepthTracker(const NuiTrackerConfig& config, UINT nWidth, UINT nHeight)
	: m_configuration(config)
	, m_error(0.0f)
	, m_numValidPoints(0)
{
	AcquireBuffers(nWidth, nHeight);
}

NuiKinfuCPUDepthTracker::~NuiKinfuCPUDepthTracker()
{
	ReleaseBuffers();
}

void	NuiKinfuCPUDepthTracker::AcquireBuffers( UINT nWidth, UINT nHeight)
{
	const NuiTrackerConfig::ITERATION_CLASS& iterations = m_configuration.iterations;
	for (UINT i = 1; i < iterations.size(); i ++)
	{
		NuiFloatImage* depths = new NuiFloatImage();
		depths->AllocateBuffer(nWidth>>i, nHeight>>i);
		m_depthsHierarchy.push_back(depths);

		NuiFloat3Image* vertices = new NuiFloat3Image();
		vertices->AllocateBuffer(nWidth>>i, nHeight>>i);
		m_verticesHierarchy.push_back(vertices);
	}
}

void	NuiKinfuCPUDepthTracker::ReleaseBuffers()
{
	for (UINT i = 1; i < m_depthsHierarchy.size(); i ++)
	{
		NuiFloatImage* depths = m_depthsHierarchy.at(i);
		SafeDelete(depths);

		NuiFloat3Image* vertices = m_verticesHierarchy.at(i);
		SafeDelete(vertices);
	}
	m_depthsHierarchy.clear();
	m_verticesHierarchy.clear();
}

void NuiKinfuCPUDepthTracker::log(const std::string& fileName) const
{
	m_configuration.log(fileName);
}

bool	NuiKinfuCPUDepthTracker::EstimatePose(
	NuiKinfuFrame* pFrame,
	NuiKinfuFeedbackFrame* pFeedbackFrame,
	NuiKinfuCameraState* pCameraState,
	Eigen::Affine3f *hint
	)
{
	if(!pFrame)
		return false;
	NuiKinfuCPUFrame* pCPUFrame = dynamic_cast<NuiKinfuCPUFrame*>(pFrame);
	if(!pCPUFrame)
		return false;

	// half sample the input depth maps into the pyramid levels
	SubSampleDepths(pCPUFrame->GetFilteredDepthBuffer());

	if(!pCameraState)
		return false;

	HierarchyDepth2vertex(pCameraState->GetCameraPos().getIntrinsics());

	if(!pFeedbackFrame)
		return false;
	NuiKinfuCPUFeedbackFrame* pCPUFeedbackFrame = dynamic_cast<NuiKinfuCPUFeedbackFrame*>(pFeedbackFrame);
	if(!pCPUFeedbackFrame)
		return false;

	return IterativeClosestPoint(
		pCPUFrame->GetVertexBuffer(),
		pCPUFeedbackFrame->GetVertexBuffer(),
		pCPUFeedbackFrame->GetNormalsBuffer(),
		pCPUFrame->GetWidth(),
		pCPUFrame->GetHeight(),
		pCameraState,
		hint);
}

void NuiKinfuCPUDepthTracker::SubSampleDepths(float* filteredDepths)
{
	// Sub sample
	float depthThreshold = m_configuration.depth_threshold;
	const int subSampleRadius = 1;
	UINT nBufferSize = (UINT)m_depthsHierarchy.size();
	for (UINT i = 0; i < nBufferSize; ++i)
	{
		NuiFloatImage* depthsDstImg = m_depthsHierarchy.at(i);
		if(!depthsDstImg)
			continue;

		float* depthsSrcBuffer = (i > 0) ? m_depthsHierarchy.at(i-1)->GetBuffer() : filteredDepths;
		float* depthsDstBuffer = depthsDstImg->GetBuffer();
		if(!depthsSrcBuffer || !depthsDstBuffer)
			continue;

		UINT rangeX = depthsDstImg->GetWidth();
		UINT rangeY = depthsDstImg->GetHeight();
#ifdef WITH_OPENMP
		#pragma omp parallel for
#endif
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
						if( nearX>=0 && nearX<(int)rangeX && nearY>=0 && nearY<(int)rangeY )
						{
							const int nearId = nearY * rangeX + nearX;
							float near = depthsSrcBuffer[nearId];
							if(near > 0.0f && (center == NAN_FLOAT || fabs(center - near) < depthThreshold))
							{
								sumDepth += near;
								sumWeight += 1;
							}
						}
					}
				}
				depthsDstBuffer[dstId] = (sumWeight > 0) ? sumDepth/sumWeight : NAN_FLOAT;
			}
		}
	}
}

void NuiKinfuCPUDepthTracker::HierarchyDepth2vertex(NuiCameraIntrinsics cameraIntrics)
{
	for (UINT i = 0; i < m_depthsHierarchy.size(); ++i)
	{
		int div = 1 << (i+1);

		// depth2vertex
		NuiFloatImage* depthsImg = m_depthsHierarchy.at(i);
		NuiFloat3Image* verticesImg = m_verticesHierarchy.at(i);
		if(!depthsImg || !verticesImg)
			continue;

		float* depthsBuffer = depthsImg->GetBuffer();
		Vector3f* verticesBuffer = verticesImg->GetBuffer();
		if(!depthsBuffer || !verticesBuffer)
			continue;

		UINT rangeX = depthsImg->GetWidth();
		UINT rangeY = depthsImg->GetHeight();
#ifdef WITH_OPENMP
		#pragma omp parallel for
#endif
		for (UINT y = 0; y < rangeY; y++)
		{
			for (UINT x = 0; x < rangeX; x++)
			{
				const UINT id = y * rangeX + x;
				float dp = depthsBuffer[id];

				if(dp != NAN_FLOAT)
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

	if (_IsNan(a))
		delta[0] = 1.0f;
	if (_IsNan(b))
		delta[0] = 0.0f;
	if (_IsNan(c))
		delta[1] = 1.0f;
	if (_IsNan(d))
		delta[1] = 0.0f;

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
bool NuiKinfuCPUDepthTracker::IterativeClosestPoint(
	Vector3f* verticesBuffer,
	Vector3f* verticesPrevBuffer,
	Vector3f* normalsPrevBuffer,
	UINT	nWidth,
	UINT	nHeight,
	NuiKinfuCameraState* pCameraState,
	Eigen::Affine3f *hint
	)
{
	if(!pCameraState)
		return false;

	const NuiCameraIntrinsics& cameraIntrinsics = pCameraState->GetCameraPos().getIntrinsics();
	const Matrix3frm& Rprev = pCameraState->GetCameraPos().getRotation();
	const Vector3f& tprev = pCameraState->GetCameraPos().getLocalTranslation();

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
	float distThreshStep = m_configuration.dist_threshold / LEVELS;
	float distThresh = m_configuration.dist_threshold + distThreshStep;

	//ScopeTime time("icp-all");
	for (int level_index = LEVELS-1; level_index>=0; --level_index)
	{
		Vector3f* vertices = verticesBuffer;
		UINT rangeX = nWidth;
		UINT rangeY = nHeight;

		if(level_index > 0)
		{
			vertices = m_verticesHierarchy.at(level_index-1)->GetBuffer();
			rangeX = m_verticesHierarchy.at(level_index-1)->GetWidth();
			rangeY = m_verticesHierarchy.at(level_index-1)->GetHeight();
		}
		if(!vertices)
			continue;

		//Vector3f* normalsBuffer = m_normals.GetBuffer();

		int iter_num = iterations[level_index].m_num;
		NuiTrackerConfig::TrackerIterationType iter_type = iterations[level_index].m_type;
		bool bShortIteration = (NuiTrackerConfig::eTracker_Iteration_Both != iter_type);
		const int numPara = bShortIteration ? 3 : 6;
		const int numParaSQ = bShortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

		// Reset params
		m_error = 1e20f;
		m_numValidPoints = 0;
		float lambda = 1.0;
		Matrix3frm lastGoodRotInv = Rinv;
		Vector3f lastGoodTrans = tcurr;
		distThresh = distThresh - distThreshStep;

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
					Vector3f vert = vertices[id];
					if(_IsNan(vert))
						continue;

					Vector3f projectedVert = Rinv * (vert - tcurr);
					Vector3f projectedPos = Rprev * projectedVert + tprev;
					Vector2f projPixel( projectedPos[0] * cameraIntrinsics.m_fx / projectedPos[2] + cameraIntrinsics.m_cx,
										projectedPos[1] * cameraIntrinsics.m_fy / projectedPos[2] + cameraIntrinsics.m_cy);
					if(projPixel[0] < 0 || (UINT)projPixel[0] >= nWidth-1 || projPixel[1] < 0 || (UINT)projPixel[1] >= nHeight-1)
						continue;

					Vector3f referenceVert = InterpolateBilinear_withHoles(verticesPrevBuffer, projPixel, nWidth);
					if(_IsNan(referenceVert))
						continue;

					Vector3f ptDiff = referenceVert - projectedVert;
					float dist = ptDiff.norm();
					if (dist > distThresh)
						continue;

					Vector3f referenceNorm = InterpolateBilinear_withHoles(normalsPrevBuffer, projPixel, nWidth);
					if(_IsNan(referenceNorm))
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

			if(!m_numValidPoints)
				break;

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

