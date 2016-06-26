#include "NuiMeshingUtil.h"
#include "NuiExperimentCPU.h"
#include "Foundation/NuiDebugMacro.h"
#include "Shape/NuiCLMappableData.h"
#include "OpenCLUtilities/NuiMappable.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLBufferFactory.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"


void NuiMeshingUtil::EvaluateMappableDataCPU(NuiCLMappableData* pCLData, bool bIsMesh, bool bOnlyShowBody)
{
	assert(pCLData);
	if(!pCLData)
		return;

//	bool bNormalEstimation = false;
//	float threshold = 0.3f;
//
//	std::vector<SgVec3f>& clPositions =
//		NuiMappableAccessor::asVectorImpl(pCLData->PositionStream())->data();
//
//	const UINT nPointNum = (UINT)clPositions.size();
//	const UINT nWidth = pCLData->GetDepthImage().GetWidth();
//	const UINT nHeight = pCLData->GetDepthImage().GetHeight();
//	const UINT16 minZ =  pCLData->GetMinDepth();
//	const UINT16 maxZ =  pCLData->GetMaxDepth();
//
//	UINT16* pDepthBuffer = pCLData->GetDepthImage().GetBuffer();
//	if( !pDepthBuffer || (nPointNum != pCLData->GetDepthImage().GetBufferNum()) )
//		return;
//
//	std::vector<unsigned int>& clPointIndices =
//		NuiMappableAccessor::asVectorImpl(pCLData->PointIndices())->data();
//	clPointIndices.clear();
//
//	std::vector<unsigned int>& clTriangleIndices =
//		NuiMappableAccessor::asVectorImpl(pCLData->TriangleIndices())->data();
//	clTriangleIndices.clear();
//
//	std::vector<unsigned int>& clWireframeIndices =
//		NuiMappableAccessor::asVectorImpl(pCLData->WireframeIndices())->data();
//	clWireframeIndices.clear();
//
//	const UINT nColorWidth = pCLData->GetColorImage().GetWidth();
//	const UINT nColorHeight = pCLData->GetColorImage().GetHeight();
//	BGRQUAD* pColorBuffer = pCLData->GetColorImage().GetBuffer();
//	BYTE* pBodyIndexBuffer = pCLData->GetBodyIndexImage().GetBuffer();
//	if(nPointNum != pCLData->GetBodyIndexImage().GetWidth()*pCLData->GetBodyIndexImage().GetHeight())
//		pBodyIndexBuffer = NULL;
//	std::vector<SgVec4f>& clColors =
//		NuiMappableAccessor::asVectorImpl(pCLData->ColorStream())->data();
//	if (nPointNum != pCLData->ColorStream().size())
//	{
//		clColors.resize(nPointNum);
//	}
//	std::vector<SgVec2f>& clUVs =
//		NuiMappableAccessor::asVectorImpl(pCLData->PatchUVStream())->data();
//
//	std::vector<SgVec3f>& clNormals =
//		NuiMappableAccessor::asVectorImpl(pCLData->NormalStream())->data();
//	if (nPointNum != clNormals.size())
//	{
//		clNormals.resize(nPointNum);
//	}
//
//	for (UINT h = 0; h < nHeight; ++h)
//	{
//		for (UINT w = 0; w < nWidth; ++w)
//		{
//			UINT idx1 = w + h * nWidth;
//
//			//bool bIsColorValid = false;
//			//if(pColorBuffer && (idx1 < clUVs.size()))
//			//{
//			//	SgVec2f uv = clUVs.at(idx1);
//			//	// Values that are negative infinity means it is an invalid depth to color mapping so we
//			//	// skip processing for this pixel
//			//	if (uv[0] != -std::numeric_limits<float>::infinity() && uv[1] != -std::numeric_limits<float>::infinity())
//			//	{
//			//		int colorX = static_cast<int>(uv[0] + 0.5f);
//			//		int colorY = static_cast<int>(uv[1] + 0.5f);
//
//			//		if ((colorX >= 0 && colorX < (int)nColorWidth) && (colorY >= 0 && colorY < (int)nColorHeight))
//			//		{
//			//			const BGRQUAD& colorValue = pColorBuffer[colorX + (colorY * nColorWidth)];
//			//			clColors.at(idx1).setValue((float)colorValue.rgbRed/255, (float)colorValue.rgbGreen/255, (float)colorValue.rgbBlue/255, 1.0f);
//			//			bIsColorValid = true;
//			//		}
//			//	}
//			//	clUVs.at(idx1).setValue(uv[0]/nColorWidth, uv[1]/nColorHeight);
//			//}
//			//if(!bIsColorValid)
//			//	clColors.at(idx1).setValue(0.0f, 0.0f, 0.0f, 0.0f);
//
//			float rPtZ1 = clPositions.at(idx1)[2];
//			bool bIsIdx1Valid = (rPtZ1 > 0.0f) && (pDepthBuffer[idx1] > minZ) && (pDepthBuffer[idx1] < maxZ);
//
//#ifdef _DEBUG
//			if(w == nWidth/2 && h == nHeight/2)
//			{
//				float xxx = clPositions.at(idx1)[0];
//				float yyy = clPositions.at(idx1)[1];
//				xxx= yyy;
//			}
//			else if(w == nWidth-1 && h == nHeight/2)
//			{
//				float xxx = clPositions.at(idx1)[0];
//				float yyy = clPositions.at(idx1)[1];
//				xxx= yyy;
//			}
//#endif
//			if(bNormalEstimation)
//			{
//				clNormals.at(idx1) = NuiExperimentCPU::estimateNormalsCovariance(clPositions, w, h, nWidth, nHeight, threshold);
//				/*if(bIsIdx1Valid)
//				{
//					SgVec3f leftVector;
//					if(w > 0)
//					{
//						UINT idxLeft = idx1-1;
//						bool bIsValid = (clPositions.at(idxLeft)[2] > 0.0f) && (pDepthBuffer[idxLeft] > minZ) && (pDepthBuffer[idxLeft] < maxZ);
//						if(bIsValid && abs(clPositions.at(idxLeft)[2] - rPtZ1) < threshold)
//							leftVector = clPositions.at(idxLeft) - clPositions.at(idx1);
//					}
//					SgVec3f rightVector;
//					if(w < nWidth-1)
//					{
//						UINT idxRight = idx1+1;
//						bool bIsValid = (clPositions.at(idxRight)[2] > 0.0f) && (pDepthBuffer[idxRight] > minZ) && (pDepthBuffer[idxRight] < maxZ);
//						if(bIsValid && abs(clPositions.at(idxRight)[2] - rPtZ1) < threshold)
//							rightVector = clPositions.at(idxRight) - clPositions.at(idx1);
//					}
//					SgVec3f upVector;
//					if(h > 0)
//					{
//						UINT idxUp = idx1-nWidth;
//						bool bIsValid = (clPositions.at(idxUp)[2] > 0.0f) && (pDepthBuffer[idxUp] > minZ) && (pDepthBuffer[idxUp] < maxZ);
//						if(bIsValid && abs(clPositions.at(idxUp)[2] - rPtZ1) < threshold)
//							upVector = clPositions.at(idxUp) - clPositions.at(idx1);
//					}
//					SgVec3f downVector;
//					if(h < nHeight-1)
//					{
//						UINT idxDown = idx1+nWidth;
//						bool bIsValid = (clPositions.at(idxDown)[2] > 0.0f) && (pDepthBuffer[idxDown] > minZ) && (pDepthBuffer[idxDown] < maxZ);
//						if(bIsValid && abs(clPositions.at(idxDown)[2] - rPtZ1) < threshold)
//							downVector = clPositions.at(idxDown) - clPositions.at(idx1);
//					}
//					SgVec3f leftRight = leftVector - rightVector;
//					SgVec3f upDown = upVector - downVector;
//					SgVec3f normal1(leftRight[1]*upDown[2]-leftRight[2]*upDown[1], leftRight[2]*upDown[0]-leftRight[0]*upDown[2], leftRight[0]*upDown[1]-leftRight[1]*upDown[0]);
//
//					SgVec3f leftUpVector;
//					if(w > 0 && h > 0)
//					{
//						UINT idxRound = idx1-1-nWidth;
//						bool bIsValid = (clPositions.at(idxRound)[2] > 0.0f) && (pDepthBuffer[idxRound] > minZ) && (pDepthBuffer[idxRound] < maxZ);
//						if(bIsValid && abs(clPositions.at(idxRound)[2] - rPtZ1) < threshold)
//							leftUpVector = clPositions.at(idxRound) - clPositions.at(idx1);
//					}
//					SgVec3f rightDownVector;
//					if(w < nWidth-1 && h < nHeight-1)
//					{
//						UINT idxRound = idx1+1+nWidth;
//						bool bIsValid = (clPositions.at(idxRound)[2] > 0.0f) && (pDepthBuffer[idxRound] > minZ) && (pDepthBuffer[idxRound] < maxZ);
//						if(bIsValid && abs(clPositions.at(idxRound)[2] - rPtZ1) < threshold)
//							rightDownVector = clPositions.at(idxRound) - clPositions.at(idx1);
//					}
//					SgVec3f rightUpVector;
//					if(w < nWidth-1 && h > 0)
//					{
//						UINT idxRound = idx1+1-nWidth;
//						bool bIsValid = (clPositions.at(idxRound)[2] > 0.0f) && (pDepthBuffer[idxRound] > minZ) && (pDepthBuffer[idxRound] < maxZ);
//						if(bIsValid && abs(clPositions.at(idxRound)[2] - rPtZ1) < threshold)
//							upVector = clPositions.at(idxRound) - clPositions.at(idx1);
//					}
//					SgVec3f leftDownVector;
//					if(w > 0 && h < nHeight-1)
//					{
//						UINT idxRound = idx1-1+nWidth;
//						bool bIsValid = (clPositions.at(idxRound)[2] > 0.0f) && (pDepthBuffer[idxRound] > minZ) && (pDepthBuffer[idxRound] < maxZ);
//						if(bIsValid && abs(clPositions.at(idxRound)[2] - rPtZ1) < threshold)
//							downVector = clPositions.at(idxRound) - clPositions.at(idx1);
//					}
//					SgVec3f upSlatedLine = leftUpVector - rightDownVector;
//					SgVec3f downSlatedLine = rightUpVector - leftDownVector;
//					SgVec3f normal2(upSlatedLine[1]*downSlatedLine[2]-upSlatedLine[2]*downSlatedLine[1], upSlatedLine[2]*downSlatedLine[0]-upSlatedLine[0]*downSlatedLine[2], upSlatedLine[0]*downSlatedLine[1]-upSlatedLine[1]*downSlatedLine[0]);
//					
//					clNormals.at(idx1) = normal1.normalized() + normal2.normalized();
//				}
//				else
//				{
//					clNormals.at(idx1).setValue(0.0f, 0.0f, 0.0f);
//				}*/
//			}
//			else
//			{
//				//memset(clNormals.data(), 1, /*sizeof(SgVec3f)**/clNormals.size());
//			}
//			if(!bIsIdx1Valid)
//				continue;
//
//			if(!bIsMesh)
//			{
//				if(pBodyIndexBuffer && bOnlyShowBody)
//				{
//					if(0xff != pBodyIndexBuffer[idx1])
//					{
//						clPointIndices.push_back(idx1);
//					}
//				}
//				else
//				{
//					clPointIndices.push_back(idx1);
//				}
//				continue;
//			}
//
//			if(w == (nWidth-1) || h == (nHeight-1))
//				continue;
//
//			UINT idx2 = w + 1 + h * nWidth;
//			UINT idx3 = w + (h + 1) * nWidth;
//			UINT idx4 = w + 1 + (h + 1) * nWidth;
//
//			float rPtZ2 = clPositions.at(idx2)[2];
//			bool bIsIdx2Valid = (rPtZ2 > 0) && (pDepthBuffer[idx2] > minZ) && (pDepthBuffer[idx2] < maxZ);
//			if(!bIsIdx2Valid)
//				continue;
//			if(pBodyIndexBuffer && bOnlyShowBody && (0xff == pBodyIndexBuffer[idx2]))
//				continue;
//
//			float rPtZ3 = clPositions.at(idx3)[2];
//			bool bIsIdx3Valid = (rPtZ3 > 0) && (pDepthBuffer[idx3] > minZ) && (pDepthBuffer[idx3] < maxZ);
//			if(!bIsIdx3Valid)
//				continue;
//			if(pBodyIndexBuffer && bOnlyShowBody && (0xff == pBodyIndexBuffer[idx3]))
//				continue;
//
//			// Build the upper triangle
//			if (abs(rPtZ1 - rPtZ2) < threshold &&
//				abs(rPtZ1 - rPtZ3) < threshold &&
//				abs(rPtZ2 - rPtZ3) < threshold)
//			{
//				clTriangleIndices.push_back(idx1);
//				clTriangleIndices.push_back(idx2);
//				clTriangleIndices.push_back(idx3);
//
//				clWireframeIndices.push_back(idx1);
//				clWireframeIndices.push_back(idx2);
//				clWireframeIndices.push_back(idx2);
//				clWireframeIndices.push_back(idx3);
//				clWireframeIndices.push_back(idx3);
//				clWireframeIndices.push_back(idx1);
//			}
//
//			float rPtZ4 = clPositions.at(idx4)[2];
//			bool bIsIdx4Valid = (rPtZ4 > 0) && (pDepthBuffer[idx4] > minZ) && (pDepthBuffer[idx4] < maxZ);
//			if(!bIsIdx4Valid)
//				continue;
//			if(pBodyIndexBuffer && bOnlyShowBody && (0xff == pBodyIndexBuffer[idx4]))
//				continue;
//
//			// Build the down triangle
//			if (abs(rPtZ4 - rPtZ2) < threshold &&
//				abs(rPtZ4 - rPtZ3) < threshold &&
//				abs(rPtZ2 - rPtZ3) < threshold)
//			{
//				clTriangleIndices.push_back(idx3);
//				clTriangleIndices.push_back(idx2);
//				clTriangleIndices.push_back(idx4);
//
//				clWireframeIndices.push_back(idx3);
//				clWireframeIndices.push_back(idx2);
//				clWireframeIndices.push_back(idx2);
//				clWireframeIndices.push_back(idx4);
//				clWireframeIndices.push_back(idx4);
//				clWireframeIndices.push_back(idx3);
//			}
//		}
//	}
//
//	if(0 == clPointIndices.size())
//	{
//		clPointIndices.resize(nPointNum);
//		for (UINT32 i = 0; i < nPointNum; ++i)
//		{
//			clPointIndices[i] = i;
//		}
//	}
//	if(0 == clTriangleIndices.size())
//	{
//		const UINT nTrianglesNum = 6 * (nWidth-1) * (nHeight-1);
//		clTriangleIndices.resize(nTrianglesNum);
//		UINT i = 0;
//		for (UINT32 h = 0; h < (nHeight-1); ++h)
//		{
//			for (UINT32 w = 0; w < (nWidth-1); ++w)
//			{
//				UINT32 idx1 = w + h * nWidth;
//				UINT32 idx2 = w + 1 + h * nWidth;
//				UINT32 idx3 = w + (h + 1) * nWidth;
//				UINT32 idx4 = w + 1 + (h + 1) * nWidth;
//				clTriangleIndices[i++] = idx1;
//				clTriangleIndices[i++] = idx2;
//				clTriangleIndices[i++] = idx3;
//				clTriangleIndices[i++] = idx3;
//				clTriangleIndices[i++] = idx2;
//				clTriangleIndices[i++] = idx4;
//			}
//		}
//	}
//
//	if(0 == clWireframeIndices.size())
//	{
//		const UINT nWiresNum = 12 * (nWidth-1) * (nHeight-1);
//		clWireframeIndices.resize(nWiresNum);
//		UINT i = 0;
//		for (UINT32 h = 0; h < (nHeight-1); ++h)
//		{
//			for (UINT32 w = 0; w < (nWidth-1); ++w)
//			{
//				UINT32 idx1 = w + h * nWidth;
//				UINT32 idx2 = w + 1 + h * nWidth;
//				UINT32 idx3 = w + (h + 1) * nWidth;
//				UINT32 idx4 = w + 1 + (h + 1) * nWidth;
//				clWireframeIndices[i++] = idx1;
//				clWireframeIndices[i++] = idx2;
//				clWireframeIndices[i++] = idx2;
//				clWireframeIndices[i++] = idx3;
//				clWireframeIndices[i++] = idx3;
//				clWireframeIndices[i++] = idx1;
//
//				clWireframeIndices[i++] = idx3;
//				clWireframeIndices[i++] = idx2;
//				clWireframeIndices[i++] = idx2;
//				clWireframeIndices[i++] = idx4;
//				clWireframeIndices[i++] = idx4;
//				clWireframeIndices[i++] = idx3;
//			}
//		}
//	}
}

void NuiMeshingUtil::NormalEstimationCL(NuiCLMappableData* pCLData)
{
	assert(pCLData);
	if(!pCLData)
		return;

	UINT nWidth = pCLData->WidthStep();
	if( 0 == nWidth )
		return;
	UINT nHeight = pCLData->GetPositionNum() / nWidth;

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();
	cl_context       context = NuiOpenCLGlobal::instance().clContext();

	// Get the kernel
	cl_kernel normalEstimationKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_ESTIMATE_NORMALS_CONVARIANCE);
	assert(normalEstimationKernel);
	if (!normalEstimationKernel)
	{
		NUI_ERROR("Get kernel 'E_ESTIMATE_NORMALS' failed!\n");
		return;
	}

	cl_kernel normalSmoothKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_SMOOTH_NORMALS);
	assert(normalSmoothKernel);
	if (!normalSmoothKernel)
	{
		NUI_ERROR("Get kernel 'E_SMOOTH_NORMALS' failed!\n");
		return;
	}

	cl_mem positionsCL =
		NuiOpenCLBufferFactory::asPosition3fBufferCL(pCLData->PositionStream());
	assert(positionsCL);
	if (!positionsCL)
	{
		NUI_ERROR("Failed to get positions of the spline item\n");
		return;
	}

	cl_mem normalsCL =
		NuiOpenCLBufferFactory::asNormal3fBufferCL(pCLData->NormalStream());
	assert(normalsCL);

	// OpenCL events
	openclutil::AutoEvent glAcquireEvent, glReleaseEvent, firstKernelEvent, secondKernelEvent;

	// Acquire OpenGL objects before use
	cl_mem glObjs[] = {
		positionsCL,
		normalsCL
	};
	openclutil::enqueueAcquireHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, glAcquireEvent);

	const float depthThreshold = 0.3f;

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(normalEstimationKernel, idx++, sizeof(cl_mem), &positionsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(normalEstimationKernel, idx++, sizeof(cl_mem), &normalsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(normalEstimationKernel, idx++, sizeof(float), &depthThreshold);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	size_t kernelGlobalSize[2] = { nWidth, nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		normalEstimationKernel,
		2,
		nullptr,
		kernelGlobalSize,
		nullptr,
		1,
		glAcquireEvent,
		firstKernelEvent
		);
	NUI_CHECK_CL_ERR(err);

	const float dotThreshold = 0.2f;//72C
	// Set kernel arguments
	idx = 0;
	err = clSetKernelArg(normalSmoothKernel, idx++, sizeof(cl_mem), &positionsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(normalSmoothKernel, idx++, sizeof(cl_mem), &normalsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(normalSmoothKernel, idx++, sizeof(float), &depthThreshold);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(normalSmoothKernel, idx++, sizeof(float), &dotThreshold);
	NUI_CHECK_CL_ERR(err);
	err = clEnqueueNDRangeKernel(
		queue,
		normalSmoothKernel,
		2,
		nullptr,
		kernelGlobalSize,
		nullptr,
		1,
		firstKernelEvent,
		secondKernelEvent
		);
	NUI_CHECK_CL_ERR(err);

	// Release OpenGL objects
	openclutil::enqueueReleaseHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 1, secondKernelEvent, glReleaseEvent);
}


void NuiMeshingUtil::SmoothPositionCL(NuiCLMappableData* pCLData)
{
	assert(pCLData);
	if(!pCLData)
		return;

	UINT nWidth = pCLData->WidthStep();
	if( 0 == nWidth )
		return;
	UINT nHeight = pCLData->GetPositionNum() / nWidth;

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();
	cl_context       context = NuiOpenCLGlobal::instance().clContext();

	cl_mem positionsCL =
		NuiOpenCLBufferFactory::asPosition3fBufferCL(pCLData->PositionStream());
	//assert(positionsCL);
	if (!positionsCL)
	{
		NUI_ERROR("Failed to get positions of the spline item\n");
		return;
	}

	// Get the kernel
	cl_kernel smoothKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_BILATERAL_FILTER);
	assert(smoothKernel);
	if (!smoothKernel)
	{
		NUI_ERROR("Get kernel 'E_ESTIMATE_NORMALS' failed!\n");
		return;
	}

	cl_mem cacheBuffer = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, nWidth*nHeight*3*sizeof(float), NULL, &err);
	NUI_CHECK_CL_ERR(err);

	// OpenCL events
	openclutil::AutoEvent glAcquireEvent, glReleaseEvent, copyKernelEvent, smoothKernelEvent;

	// Acquire OpenGL objects before use
	cl_mem glObjs[] = {
		positionsCL
	};
	openclutil::enqueueAcquireHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, glAcquireEvent);

	err = clEnqueueCopyBuffer(queue, positionsCL, cacheBuffer, 0, 0, nWidth*nHeight*3*sizeof(float), 1, glAcquireEvent, copyKernelEvent);
	NUI_CHECK_CL_ERR(err);

	const float depthThreshold = 0.3f;
	const float truncMin = 0.4f;
	const float truncMax = 4.2f;

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(smoothKernel, idx++, sizeof(cl_mem), &cacheBuffer);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(smoothKernel, idx++, sizeof(cl_mem), &positionsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(smoothKernel, idx++, sizeof(float), &depthThreshold);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(smoothKernel, idx++, sizeof(float), &truncMin);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(smoothKernel, idx++, sizeof(float), &truncMax);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	size_t kernelGlobalSize[2] = { nWidth, nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		smoothKernel,
		2,
		nullptr,
		kernelGlobalSize,
		nullptr,
		1,
		copyKernelEvent,
		smoothKernelEvent
		);
	NUI_CHECK_CL_ERR(err);

	// Release OpenGL objects
	openclutil::enqueueReleaseHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 1, smoothKernelEvent, glReleaseEvent);

	if (cacheBuffer) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(cacheBuffer);
		NUI_CHECK_CL_ERR(err);
	}

#ifdef _DEBUG
	clFinish(queue);

	cl_ulong time_start, time_end, total_time;
	clGetEventProfilingInfo(
		*smoothKernelEvent,
		CL_PROFILING_COMMAND_START,
		sizeof(time_start),
		&time_start,
		NULL
		);
	clGetEventProfilingInfo(
		*smoothKernelEvent,
		CL_PROFILING_COMMAND_END,
		sizeof(time_end),
		&time_end,
		NULL
		);

	total_time = time_end - time_start;
#endif
}


void NuiMeshingUtil::CalcColorCL(NuiCLMappableData* pCLData)
{
	assert(pCLData);
	if(!pCLData)
		return;

	UINT nWidth = pCLData->WidthStep();
	if( 0 == nWidth )
		return;
	UINT nHeight = pCLData->GetPositionNum() / nWidth;

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();
	cl_context       context = NuiOpenCLGlobal::instance().clContext();

	// Get the kernel
	cl_kernel grabKernel =
		NuiOpenCLKernelManager::instance().acquireKernel(E_ESTIMATE_NORMALS);

	assert(grabKernel);
	if (!grabKernel)
	{
		NUI_ERROR("Get kernel 'E_CALC_NORMAL' failed!\n");
		return;
	}

	cl_mem positionsCL =
		NuiOpenCLBufferFactory::asPosition3fBufferCL(pCLData->PositionStream());
	assert(positionsCL);
	if (!positionsCL)
	{
		NUI_ERROR("Failed to get positions of the spline item\n");
		return;
	}

	cl_mem uvsCL =
		NuiOpenCLBufferFactory::asPatch2fBufferCL(pCLData->PatchUVStream());
	assert(uvsCL);
	cl_mem pointIndicesCL =
		NuiOpenCLBufferFactory::asUInt32IndexBufferCL(pCLData->PointIndices());
	assert(pointIndicesCL);
	cl_mem colorsCL =
		NuiOpenCLBufferFactory::asColor4fBufferCL(pCLData->ColorStream());
	assert(colorsCL);
	cl_mem normalsCL =
		NuiOpenCLBufferFactory::asNormal3fBufferCL(pCLData->NormalStream());
	assert(normalsCL);

	// OpenCL events
	openclutil::AutoEvent glAcquireEvent, glReleaseEvent, kernelEvent;

	// Acquire OpenGL objects before use
	cl_mem glObjs[] = {
		pointIndicesCL,
		positionsCL,
		uvsCL,
		colorsCL,
		normalsCL
	};
	openclutil::enqueueAcquireHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 0, nullptr, glAcquireEvent);

	//const float distanceUnit = 100;

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(grabKernel, idx++, sizeof(cl_mem), &pointIndicesCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(grabKernel, idx++, sizeof(cl_mem), &positionsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(grabKernel, idx++, sizeof(cl_mem), &uvsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(grabKernel, idx++, sizeof(cl_mem), &colorsCL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(grabKernel, idx++, sizeof(cl_mem), &normalsCL);
	NUI_CHECK_CL_ERR(err);
	/*err = clSetKernelArg(grabKernel, idx++, sizeof(float), &distanceUnit);
	NUI_CHECK_CL_ERR(err);*/

	// Run kernel to calculate 
	size_t kernelGlobalSize[2] = { nWidth, nHeight };
	err = clEnqueueNDRangeKernel(
		queue,
		grabKernel,
		2,
		nullptr,
		kernelGlobalSize,
		nullptr,
		1,
		glAcquireEvent,
		kernelEvent
		);
	NUI_CHECK_CL_ERR(err);

	// Release OpenGL objects
	openclutil::enqueueReleaseHWObjects(
		sizeof(glObjs) / sizeof(cl_mem), glObjs, 1, kernelEvent, glReleaseEvent);
}