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

#define NAN_FLOAT -std::numeric_limits<float>::max()

typedef Eigen::Vector2f Vector2f;

bool _IsNan(Vector3f data) { return (NAN_FLOAT == data.x) || (NAN_FLOAT == data.y) ||(NAN_FLOAT == data.z); }

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
					if(_IsNan(left) || fabs(center.z - left.z) > depthThreshold)
						left = center;
				}
				Vector3f right = center;
				if(x < m_nWidth-1)
				{
					right = verticesBuffer[centerId+1];
					if(_IsNan(right) || fabs(center.z - right.z) > depthThreshold)
						right = center;
				}
				Vector3f up = center;
				if(y > 0)
				{
					up = verticesBuffer[centerId-m_nWidth];
					if(_IsNan(up) || fabs(center.z - up.z) > depthThreshold)
						up = center;
				}
				Vector3f down = center;
				if(y < m_nHeight-1)
				{
					down = verticesBuffer[centerId+m_nWidth];
					if(_IsNan(down) || fabs(center.z - down.z) > depthThreshold)
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
				if(outNormal.x == 0.0f && outNormal.y == 0.0f && outNormal.z == 0.0f)
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

///////////////////////////////////////////////////////////////////////////////////////////
// Iterative Closest Point
bool NuiKinfuCPUDepthTracker::IterativeClosestPoint(NuiKinfuCameraState* pCameraState, Eigen::Affine3f *hint)
{
	if(!pCameraState)
		return false;

	const NuiCameraPos& cameraPos = pCameraState->GetCameraPos();
	const NuiCameraIntrinsics& cameraIntrinsics = pCameraState->GetCameraPos().getIntrinsics();

	Matrix3frm Rcurr;
	Vector3f tcurr;
	if(hint)
	{
		Rcurr = hint->rotation().matrix();
		tcurr = hint->translation().matrix();
	}
	else
	{
		Rcurr = cameraPos.getRotation(); // tranform to global coo for ith camera pose
		tcurr = cameraPos.getTranslation();
	}

	Matrix3frm Rprev = cameraPos.getRotation();
	Vector3f tprev = cameraPos.getTranslation();

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

		m_numValidPoints = 0;
		float sumF = 0.0f;
		float sumHessian[6 * 6], sumNabla[6];
		memset(sumHessian, 0, sizeof(float) * numParaSQ);
		memset(sumNabla, 0, sizeof(float) * numPara);

		for (int iter = 0; iter < iter_num; ++iter)
		{
			//
			for (UINT y = 0; y < rangeY; y ++)
			{
				for (UINT x = 0; x < rangeX; x ++)
				{
					const UINT id = y * rangeX + x;
					Vector3f vert = verticesBuffer[id];
					if(NAN_FLOAT == vert.x || NAN_FLOAT == vert.y || NAN_FLOAT == vert.z)
						continue;

					Vector3f projectedVert = Rcurr * vert + tcurr;
					Vector3f projectedPos = Rprev.inverse() * (projectedVert - tprev);
					Vector2f projPixel( projectedPos.x * cameraIntrinsics.m_fx / projectedPos.z + cameraIntrinsics.m_cx,
										projectedPos.y * cameraIntrinsics.m_fy / projectedPos.z + cameraIntrinsics.m_cy);
					if(projPixel.x < 0 || projPixel.x >= m_nWidth || projPixel.y < 0 || projPixel.y >= m_nHeight)
						continue;

					Vector3f referenceVert = interpolateBilinear_withHoles(m_verticesPrev.GetBuffer(), projPixel, m_verticesPrev.GetWidth(), m_verticesPrev.GetHeight());
					if(NAN_FLOAT == referenceVert.x || NAN_FLOAT == referenceVert.y || NAN_FLOAT == referenceVert.z)
						continue;

					Vector3f ptDiff = referenceVert - projectedVert;
					float dist = ptDiff.x * ptDiff.x + ptDiff.y * ptDiff.y + ptDiff.z * ptDiff.z;
					if (dist > m_configuration.dist_threshold)
						continue;

					Vector3f referenceNorm = interpolateBilinear_withHoles(m_normalsPrev.GetBuffer(), projPixel, m_normalsPrev.GetWidth(), m_normalsPrev.GetHeight());
					if(NAN_FLOAT == referenceNorm.x || NAN_FLOAT == referenceNorm.y || NAN_FLOAT == referenceNorm.z)
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
						A[0] = row0.x;
						A[1] = row0.y;
						A[2] = row0.z;
					}
					else if (NuiTrackerConfig::eTracker_Iteration_Translation == iter_type)
					{
						A[0] = referenceNorm.x;
						A[1] = referenceNorm.y;
						A[2] = referenceNorm.z;
					}
					else
					{
						A[0] = row0.x;
						A[1] = row0.y;
						A[2] = row0.z;
						A[3] = referenceNorm.x;
						A[4] = referenceNorm.y;
						A[5] = referenceNorm.z;
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
					m_error = (m_numValidPoints > 100) ? sqrt(sumF) / m_numValidPoints : 1e5f;
				}
			}

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
		clVertices[i][0] = verticesBuffer[i].x;
		clVertices[i][1] = verticesBuffer[i].y;
		clVertices[i][2] = verticesBuffer[i].z;
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

