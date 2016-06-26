#include "stdafx.h"

#include "NuiCompositeFrame.h"

#include "Shape\NuiCLMappableData.h"
#include "Shape\NuiMeshShape.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"

#include <boost/smart_ptr.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


namespace NuiFrameUtilities
{
	static const int	cDepthFps = 30;
	static const int	cHalfADepthFrameMs = (1000 / cDepthFps) * 2;

	bool FrameToCompoundImage (NuiCompositeFrame* pCompositeFrame)
	{
		assert(pCompositeFrame);
		if(!pCompositeFrame)
			return false;

		BGRQUAD* pColorBuffer = pCompositeFrame->m_colorFrame.GetBuffer();
		if(!pColorBuffer)
			return false;

		UINT16* pDepthBuffer = pCompositeFrame->m_depthFrame.GetBuffer();
		if(!pDepthBuffer)
			return false;

		if(std::abs(pCompositeFrame->m_depthFrame.GetTimeStamp() - pCompositeFrame->m_colorFrame.GetTimeStamp()) > cHalfADepthFrameMs)
			return false;

		UINT nDepthWidth = pCompositeFrame->m_depthFrame.GetWidth();
		UINT nDepthHeight = pCompositeFrame->m_depthFrame.GetHeight();
		UINT nDepthPointNum = nDepthWidth * nDepthHeight;

		UINT nColorWidth = pCompositeFrame->m_depthMapFrame.GetWidth();
		UINT nColorHeight = pCompositeFrame->m_depthMapFrame.GetHeight();
		UINT nColorPointNum = nColorWidth * nColorHeight;
		if(!nColorPointNum)
			return false;
		
	//	DepthSpacePoint* pColorToDepth = pCompositeFrame->m_depthMapFrame.GetBuffer();
	//	if (pColorToDepth)
	//	{
	//		BYTE* pBodyIndexBuffer = NULL;
	//		//assert((m_nBodyIndexWidth == m_nDepthWidth) && (m_nBodyIndexHeight == m_nDepthHeight));
	//		if(std::abs(pCompositeFrame->m_depthFrame.GetTimeStamp() - pCompositeFrame->m_bodyIndexFrame.GetTimeStamp()) < cHalfADepthFrameMs)
	//			pBodyIndexBuffer = pCompositeFrame->m_bodyIndexFrame.GetBuffer();

	//		NuiCompoundPixel* pCachedPixels = pCompositeFrame->m_compoundImageFrame.AllocatePixels(nColorWidth, nColorHeight);
	//		assert(pCachedPixels);
	//		pCompositeFrame->m_compoundImageFrame.WriteFrameLock();
	//		// loop over output pixels
	//#pragma omp parallel for schedule(dynamic)
	//		for (UINT colorIndex = 0; colorIndex < nColorPointNum; ++colorIndex)
	//		{
	//			NuiCompoundPixel* pPixel = pCachedPixels + colorIndex;
	//			if(!pPixel)
	//				continue;

	//			pPixel->fColor = pColorBuffer[colorIndex];

	//			const DepthSpacePoint& rPt = pColorToDepth[colorIndex];
	//			// Values that are negative infinity means it is an invalid depth to color mapping so we
	//			// skip processing for this pixel
	//			if (rPt.X != -std::numeric_limits<float>::infinity() && rPt.Y != -std::numeric_limits<float>::infinity())
	//			{
	//				int depthX = static_cast<int>(rPt.X + 0.5f);
	//				int depthY = static_cast<int>(rPt.Y + 0.5f);

	//				if ((depthX >= 0 && depthX < (int)nDepthWidth) && (depthY >= 0 && depthY < (int)nDepthHeight))
	//				{
	//					UINT depthIndex = depthX + depthY * nDepthWidth;
	//					pPixel->fDepth = pDepthBuffer[depthIndex];

	//					if(pBodyIndexBuffer)
	//					{
	//						pPixel->fBodyIndex = pBodyIndexBuffer[depthIndex];
	//					}
	//					else
	//					{
	//						pPixel->fBodyIndex = 0xff;
	//					}
	//				}
	//				else
	//				{
	//					pPixel->fDepth = 0;
	//					pPixel->fBodyIndex = 0xff;
	//				}
	//			}
	//		}
	//		pCompositeFrame->m_compoundImageFrame.WriteFrameUnlock();
	//	}

		return true;
	}

	
	bool FrameToPointCloud (NuiCompositeFrame* pCompositeFrame)
	{
		assert(pCompositeFrame);
		if(!pCompositeFrame)
			return false;

		UINT16* pDepthBuffer = pCompositeFrame->m_depthFrame.GetBuffer();
		if(!pDepthBuffer)
			return false;

		UINT nDepthWidth = pCompositeFrame->m_depthFrame.GetWidth();
		UINT nDepthHeight = pCompositeFrame->m_depthFrame.GetHeight();
		UINT nDepthPointNum = nDepthWidth * nDepthHeight;
		if(!nDepthPointNum)
			return false;
		
		CameraSpacePoint* pDepthToCamera = pCompositeFrame->m_cameraMapFrame.GetBuffer();
		if (pDepthToCamera)
		{
			UINT nColorWidth = pCompositeFrame->m_colorFrame.GetWidth();
			UINT nColorHeight = pCompositeFrame->m_colorFrame.GetHeight();
			UINT nColorPointNum = nColorWidth * nColorHeight;
			BGRQUAD* pColorBuffer = pCompositeFrame->m_colorFrame.GetBuffer();
			ColorSpacePoint* pDepthToColor = NULL;
			if(pColorBuffer && nColorPointNum > 0 /*&&
				(std::abs(m_depthFrameBuffer.GetTimeStamp() - m_colorFrameBuffer.GetTimeStamp()) < cHalfADepthFrameMs)*/)
			{
				pCompositeFrame->m_pointCloudFrame.SetColorImage(pCompositeFrame->m_colorFrame.GetImage());
				// map to color space
				ColorSpacePoint* pDepthToColor = pCompositeFrame->m_colorMapFrame.GetBuffer();
				if(!pDepthToColor)
					pColorBuffer = nullptr;
			}
			else
			{
				pColorBuffer = nullptr;
			}

			BYTE* pBodyIndexBuffer = nullptr;
			//assert((m_nBodyIndexWidth == m_nDepthWidth) && (m_nBodyIndexHeight == m_nDepthHeight));
			if(std::abs(pCompositeFrame->m_depthFrame.GetTimeStamp() - pCompositeFrame->m_bodyIndexFrame.GetTimeStamp()) < cHalfADepthFrameMs)
				pBodyIndexBuffer = pCompositeFrame->m_bodyIndexFrame.GetBuffer();

			UINT16 minZ = pCompositeFrame->m_depthFrame.GetMinDepth();
			UINT16 maxZ = pCompositeFrame->m_depthFrame.GetMaxDepth();

			NuiDevicePoint* pPoints = pCompositeFrame->m_pointCloudFrame.AllocatePoints(nDepthPointNum);
			pCompositeFrame->m_pointCloudFrame.SetWidthStep(nDepthWidth);
			assert(pPoints);
			pCompositeFrame->m_pointCloudFrame.WriteFrameLock();
			// loop over output pixels
	#pragma omp parallel for schedule(dynamic)
			for (UINT depthIndex = 0; depthIndex < nDepthPointNum; ++depthIndex)
			{
				NuiDevicePoint* pCachePt = pPoints + depthIndex;
				if(!pCachePt)
					continue;
				const CameraSpacePoint& rPt = pDepthToCamera[depthIndex];
				pCachePt->fIsValid = (rPt.Z > 0) && (pDepthBuffer[depthIndex] > minZ) && (pDepthBuffer[depthIndex] < maxZ);
				if(pCachePt->fIsValid)
				{
					pCachePt->fVertex.x = rPt.X;
					pCachePt->fVertex.y = rPt.Y;
					pCachePt->fVertex.z = rPt.Z;
				}
				else
				{
					pCachePt->fVertex.x = 0.0f;
					pCachePt->fVertex.y = 0.0f;
					pCachePt->fVertex.z = 0.0f;
				}
				bool bIsColorValid = false;
				if(pColorBuffer && pDepthToColor)
				{
					ColorSpacePoint p = pDepthToColor[depthIndex];
					// Values that are negative infinity means it is an invalid depth to color mapping so we
					// skip processing for this pixel
					if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
					{
						int colorX = static_cast<int>(p.X + 0.5f);
						int colorY = static_cast<int>(p.Y + 0.5f);

						if ((colorX >= 0 && colorX < (int)nColorWidth) && (colorY >= 0 && colorY < (int)nColorHeight))
						{
							bIsColorValid = true;
							pCachePt->fBGRA = pColorBuffer[colorX + (colorY * nColorWidth)];
							pCachePt->fColorSpaceU = (float)colorX / nColorWidth;
							pCachePt->fColorSpaceV = (float)colorY / nColorHeight;

							pCachePt->fNormal_x = 0.0f;
							pCachePt->fNormal_y = 0.0f;
							pCachePt->fNormal_z = -1.0f;
						}
					}
				}
				if(!bIsColorValid)
				{
					pCachePt->fBGRA.rgbBlue = 0;
					pCachePt->fBGRA.rgbGreen = 0;
					pCachePt->fBGRA.rgbRed = 0;
					pCachePt->fBGRA.rgbReserved = 0;
					pCachePt->fColorSpaceU = -1.0f;
					pCachePt->fColorSpaceV = -1.0f;
				}

				if(pBodyIndexBuffer)
				{
					pCachePt->fBodyIndex = pBodyIndexBuffer[depthIndex];
				}
				else
				{
					pCachePt->fBodyIndex = 0xff;
				}
			}
			pCompositeFrame->m_pointCloudFrame.WriteFrameUnlock();
		}
		else
		{
			std::cerr << "Error : Failed to acquire depth to camera mapping in frameToPointCloud." << std::endl;
		}

		return true;
	}

	bool	CameraSpacePointsToMappableData(NuiCompositeFrame* pCompositeFrame, NuiCLMappableData* pData)
	{
		if(!pData || !pCompositeFrame)
			return false;

		const UINT nFrameWidth = pCompositeFrame->m_cameraMapFrame.GetWidth();
		const UINT nFrameHeight = pCompositeFrame->m_cameraMapFrame.GetHeight();
		const UINT nFramePointsNum = nFrameWidth * nFrameHeight;
		CameraSpacePoint* pDepthToCamera = pCompositeFrame->m_cameraMapFrame.GetBuffer();
		if(!pDepthToCamera)
			return false;
		
		std::shared_ptr<NuiVectorMappableImpl3f> positions = NuiMappableAccessor::asVectorImpl(pData->PositionStream());
		if(positions->data().size() != nFramePointsNum)
			positions->data().resize(nFramePointsNum);
		pCompositeFrame->m_cameraMapFrame.ReadFrameLock();
		errno_t err = memcpy_s(positions->data().data(), sizeof(SgVec3f)*nFramePointsNum, pDepthToCamera, sizeof(CameraSpacePoint)*nFramePointsNum);
		pCompositeFrame->m_cameraMapFrame.ReadFrameUnlock();
		if(0 != err)
		{
			std::cerr << "Error : Failed to copy the position buffer in frameToCLData." << std::endl;
			return false;
		}

		// Normals
		if (nFramePointsNum != pData->NormalStream().size())
		{
			std::vector<SgVec3f>& clNormals =
				NuiMappableAccessor::asVectorImpl(pData->NormalStream())->data();
			clNormals.resize(nFramePointsNum);
		}

		return true;
	}

	bool	ColorMapToMappableData(NuiCompositeFrame* pCompositeFrame, NuiCLMappableData* pData)
	{
		if(!pData || !pCompositeFrame)
			return false;

		// Patch UV
		const UINT nColorMapWidth = pCompositeFrame->m_colorMapFrame.GetWidth();
		const UINT nColorMapHeight = pCompositeFrame->m_colorMapFrame.GetHeight();
		const UINT nColorMapNum = nColorMapWidth * nColorMapHeight;
		ColorSpacePoint* pDepthToColor = pCompositeFrame->m_colorMapFrame.GetBuffer();
		if(!pDepthToColor)
			return false;
		
		std::shared_ptr<NuiVectorMappableImpl2f> uvs = NuiMappableAccessor::asVectorImpl(pData->PatchUVStream());
		if(uvs->data().size() != nColorMapNum)
			uvs->data().resize(nColorMapNum);
		pCompositeFrame->m_colorMapFrame.ReadFrameLock();
		errno_t err = memcpy_s(uvs->data().data(), sizeof(SgVec2f)*nColorMapNum, pDepthToColor, sizeof(ColorSpacePoint)*nColorMapNum);
		pCompositeFrame->m_colorMapFrame.ReadFrameUnlock();
		if(0 != err)
		{
			std::cerr << "Error : Failed to copy the uv buffer in frameToCLData." << std::endl;
			return false;
		}

		// Colors
		std::vector<SgVec4f>& clColors =
			NuiMappableAccessor::asVectorImpl(pData->ColorStream())->data();
		clColors.clear();

		return true;
	}
	
	bool	FrameToMappableData(NuiCompositeFrame* pCompositeFrame, NuiCLMappableData* pData, int indexFlags, bool bOnlyShowBody, float depthThreshold)
	{
		if(!pData || !pCompositeFrame)
			return false;

		if( !CameraSpacePointsToMappableData(pCompositeFrame, pData) )
			return false;

		if( !ColorMapToMappableData(pCompositeFrame, pData) )
			return false;

		pData->SetStreamDirty(true);

		// For color texture
		//if(std::abs(m_depthFrame.GetTimeStamp() - m_colorFrame.GetTimeStamp()) < cHalfADepthFrameMs)
		{
			pCompositeFrame->m_colorFrame.ReadFrameLock();
			pData->SetColorImage( pCompositeFrame->m_colorFrame.GetImage() );
			pCompositeFrame->m_colorFrame.ReadFrameUnlock();
		}

		// Camera
		pData->SetCameraParams( pCompositeFrame->GetCameraParams() );
	
		// Write Index data
		const UINT nFrameWidth = pCompositeFrame->m_cameraMapFrame.GetWidth();
		const UINT nFrameHeight = pCompositeFrame->m_cameraMapFrame.GetHeight();
		const UINT nFramePointsNum = nFrameWidth * nFrameHeight;
		CameraSpacePoint* pDepthToCamera = pCompositeFrame->m_cameraMapFrame.GetBuffer();
		UINT16* pDepthBuffer = pCompositeFrame->m_depthFrame.GetBuffer();
		BYTE* pBodyIndexBuffer = pCompositeFrame->m_bodyIndexFrame.GetBuffer();
		const UINT16 minZ = pCompositeFrame->m_depthFrame.GetMinDepth();
		const UINT16 maxZ = pCompositeFrame->m_depthFrame.GetMaxDepth();
		if(!pDepthToCamera || !pDepthBuffer)
			return false;

		std::vector<unsigned int>& pointIndices =
			NuiMappableAccessor::asVectorImpl(pData->PointIndices())->data();
		pointIndices.clear();

		std::vector<unsigned int>& triangleIndices =
			NuiMappableAccessor::asVectorImpl(pData->TriangleIndices())->data();
		triangleIndices.clear();

		std::vector<unsigned int>& wireframeIndices =
			NuiMappableAccessor::asVectorImpl(pData->WireframeIndices())->data();
		wireframeIndices.clear();

		const bool bNeedPointIndex = (indexFlags & NuiCLMappableData::E_MappableData_Point) ? true : false;
		const bool bNeedTriangleIndex = (indexFlags & NuiCLMappableData::E_MappableData_Triangle) ? true : false;
		const bool bNeedWireframeIndex = (indexFlags & NuiCLMappableData::E_MappableData_Wireframe) ? true : false;

		SgVec3f boundingBoxMin( std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() );
		SgVec3f boundingBoxMax( -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max() );

#pragma omp parallel for schedule(dynamic)
		for (UINT h = 0; h < nFrameHeight; ++h)
		{
			for (UINT w = 0; w < nFrameWidth; ++w)
			{
				UINT idx1 = w + h * nFrameWidth;

				float rPtZ1 = pDepthToCamera[idx1].Z;
				bool bIsIdx1Valid = (rPtZ1 > 0.0f) && pDepthBuffer && (pDepthBuffer[idx1] > minZ) && (pDepthBuffer[idx1] < maxZ);

				if(!bIsIdx1Valid)
					continue;

				if(pBodyIndexBuffer && bOnlyShowBody && (0xff == pBodyIndexBuffer[idx1]))
					continue;

				// compute bounding box
				if ( pDepthToCamera[idx1].X < boundingBoxMin[0] ) boundingBoxMin[0] = pDepthToCamera[idx1].X;
				if ( pDepthToCamera[idx1].Y < boundingBoxMin[1] ) boundingBoxMin[1] = pDepthToCamera[idx1].Y;
				if ( pDepthToCamera[idx1].Z < boundingBoxMin[2] ) boundingBoxMin[2] = pDepthToCamera[idx1].Z;
				if ( pDepthToCamera[idx1].X > boundingBoxMax[0] ) boundingBoxMax[0] = pDepthToCamera[idx1].X;
				if ( pDepthToCamera[idx1].Y > boundingBoxMax[1] ) boundingBoxMax[1] = pDepthToCamera[idx1].Y;
				if ( pDepthToCamera[idx1].Z > boundingBoxMax[2] ) boundingBoxMax[2] = pDepthToCamera[idx1].Z;
			
				if(bNeedPointIndex)
				{
					pointIndices.push_back(idx1);
				}

				if(bNeedTriangleIndex || bNeedWireframeIndex)
				{
					if(w == (nFrameWidth-1) || h == (nFrameHeight-1))
						continue;

					UINT idx2 = w + 1 + h * nFrameWidth;
					UINT idx3 = w + (h + 1) * nFrameWidth;
					UINT idx4 = w + 1 + (h + 1) * nFrameWidth;

					float rPtZ2 = pDepthToCamera[idx2].Z;
					bool bIsIdx2Valid = (rPtZ2 > 0)&& pDepthBuffer  && (pDepthBuffer[idx2] > minZ) && (pDepthBuffer[idx2] < maxZ);
					if(!bIsIdx2Valid)
						continue;
					if(pBodyIndexBuffer && bOnlyShowBody && (0xff == pBodyIndexBuffer[idx2]))
						continue;

					float rPtZ3 = pDepthToCamera[idx3].Z;
					bool bIsIdx3Valid = (rPtZ3 > 0) && pDepthBuffer && (pDepthBuffer[idx3] > minZ) && (pDepthBuffer[idx3] < maxZ);
					if(!bIsIdx3Valid)
						continue;
					if(pBodyIndexBuffer && bOnlyShowBody && (0xff == pBodyIndexBuffer[idx3]))
						continue;

					// Build the upper triangle
					if (abs(rPtZ1 - rPtZ2) < depthThreshold &&
						abs(rPtZ1 - rPtZ3) < depthThreshold &&
						abs(rPtZ2 - rPtZ3) < depthThreshold)
					{
						if(bNeedTriangleIndex)
						{
							triangleIndices.push_back(idx1);
							triangleIndices.push_back(idx2);
							triangleIndices.push_back(idx3);
						}
						
						if(bNeedWireframeIndex)
						{
							wireframeIndices.push_back(idx1);
							wireframeIndices.push_back(idx2);
							wireframeIndices.push_back(idx2);
							wireframeIndices.push_back(idx3);
							wireframeIndices.push_back(idx3);
							wireframeIndices.push_back(idx1);
						}
					}

					float rPtZ4 = pDepthToCamera[idx4].Z;
					bool bIsIdx4Valid = (rPtZ4 > 0) && pDepthBuffer && (pDepthBuffer[idx4] > minZ) && (pDepthBuffer[idx4] < maxZ);
					if(!bIsIdx4Valid)
						continue;
					if(pBodyIndexBuffer && bOnlyShowBody && (0xff == pBodyIndexBuffer[idx4]))
						continue;

					// Build the down triangle
					if (abs(rPtZ4 - rPtZ2) < depthThreshold &&
						abs(rPtZ4 - rPtZ3) < depthThreshold &&
						abs(rPtZ2 - rPtZ3) < depthThreshold)
					{
						if(bNeedTriangleIndex)
						{
							triangleIndices.push_back(idx3);
							triangleIndices.push_back(idx2);
							triangleIndices.push_back(idx4);
						}

						if(bNeedWireframeIndex)
						{
							wireframeIndices.push_back(idx3);
							wireframeIndices.push_back(idx2);
							wireframeIndices.push_back(idx2);
							wireframeIndices.push_back(idx4);
							wireframeIndices.push_back(idx4);
							wireframeIndices.push_back(idx3);
						}
					}
				}
			}
		}
		pData->SetBoundingBox(boundingBoxMin, boundingBoxMax);
		pData->SetIndexingDirty(true);
		
		return true;
	}

	bool	FrameToMesh(NuiCompositeFrame* pCompositeFrame, NuiMeshShape* pMesh, bool bOnlyShowBody, float depthThreshold)
	{
		if(!pMesh || !pCompositeFrame)
			return false;

		const UINT nFrameWidth = pCompositeFrame->m_cameraMapFrame.GetWidth();
		const UINT nFrameHeight = pCompositeFrame->m_cameraMapFrame.GetHeight();
		const UINT nFramePointsNum = nFrameWidth * nFrameHeight;
		CameraSpacePoint* pDepthToCamera = pCompositeFrame->m_cameraMapFrame.GetBuffer();
		if(!pDepthToCamera)
			return false;

		boost::scoped_array<SgVec3f> points(new SgVec3f[nFramePointsNum]);
		pCompositeFrame->m_cameraMapFrame.ReadFrameLock();
		errno_t err = memcpy_s(points.get(), sizeof(SgVec3f)*nFramePointsNum, pDepthToCamera, sizeof(CameraSpacePoint)*nFramePointsNum);
		pCompositeFrame->m_cameraMapFrame.ReadFrameUnlock();
		if(0 != err)
		{
			std::cerr << "Error : Failed to copy the position buffer in frameToMesh." << std::endl;
			return false;
		}

		// Save color texture
		const UINT nColorWidth = pCompositeFrame->m_colorFrame.GetWidth();
		const UINT nColorHeight = pCompositeFrame->m_colorFrame.GetHeight();
		IplImage*	pTextureImg = cvCreateImage(cvSize(nColorWidth,nColorHeight),IPL_DEPTH_32F ,3);
		pCompositeFrame->m_colorFrame.ReadFrameLock();
		BGRQUAD* pColors = pCompositeFrame->m_colorFrame.GetBuffer();
		if(pColors)
		{
			for (UINT y = 0; y < nColorHeight; y ++)
			{
				for (UINT x = 0; x < nColorWidth; x ++)
				{
					UINT index = x + y * nColorWidth;
					BGRQUAD* pValue = pColors + index;
					CvScalar color = CvScalar(pValue->rgbGreen, pValue->rgbBlue, pValue->rgbRed);
					cvSet2D(pTextureImg, y, x, color);
				}
			}
		}
		pCompositeFrame->m_colorFrame.ReadFrameUnlock();
		std::string textureImagePath = NuiOpenCLGlobal::instance().tmpDir() + "NuiMeshTexture.jpg";
		//cvSaveImage(NuiOpenCLGlobal::instance().tmpDir().c_str(), pTextureImg);
		cvSaveImage(textureImagePath.c_str(), pTextureImg);
		cvReleaseImage(&pTextureImg);

		// Patch UV
		boost::scoped_array<SgVec2f> uvs(NULL);
		const UINT nColorMapWidth = pCompositeFrame->m_colorMapFrame.GetWidth();
		const UINT nColorMapHeight = pCompositeFrame->m_colorMapFrame.GetHeight();
		const UINT nColorMapNum = nColorMapWidth * nColorMapHeight;
		ColorSpacePoint* pDepthToColor = pCompositeFrame->m_colorMapFrame.GetBuffer();
		if(pDepthToColor)
		{
			uvs.reset(new SgVec2f[nColorMapNum]);
			pCompositeFrame->m_colorMapFrame.ReadFrameLock();
			errno_t err = memcpy_s(uvs.get(), sizeof(SgVec2f)*nColorMapNum, pDepthToColor, sizeof(ColorSpacePoint)*nColorMapNum);
			pCompositeFrame->m_colorMapFrame.ReadFrameUnlock();
			if(0 != err)
			{
				std::cerr << "Error : Failed to copy the uv buffer in frameToMesh." << std::endl;
			}
		}
		else
		{
			std::cerr << "Error : Failed to acquire color to camera mapping in frameToMesh." << std::endl;
		}

		// Depth
		boost::scoped_array<UINT16> depths(NULL);
		UINT16* pDepthBuffer = pCompositeFrame->m_depthFrame.GetBuffer();
		if(pDepthBuffer)
		{
			depths.reset(new UINT16[nFramePointsNum]);
			pCompositeFrame->m_depthFrame.ReadFrameLock();
			errno_t err = memcpy_s(depths.get(), sizeof(UINT16)*nFramePointsNum, pDepthBuffer, sizeof(UINT16)*nFramePointsNum);
			pCompositeFrame->m_depthFrame.ReadFrameUnlock();
			if(0 != err)
			{
				std::cerr << "Error : Failed to copy the depth buffer in frameToMesh." << std::endl;
			}
		}

		// Body index
		boost::scoped_array<BYTE> bodyIndices(NULL);
		BYTE* pBodyIndexBuffer = pCompositeFrame->m_bodyIndexFrame.GetBuffer();
		if(pBodyIndexBuffer)
		{
			bodyIndices.reset(new BYTE[nFramePointsNum]);
			pCompositeFrame->m_bodyIndexFrame.ReadFrameLock();
			errno_t err = memcpy_s(bodyIndices.get(), sizeof(BYTE)*nFramePointsNum, pBodyIndexBuffer, sizeof(BYTE)*nFramePointsNum);
			pCompositeFrame->m_bodyIndexFrame.ReadFrameUnlock();
			if(0 != err)
			{
				std::cerr << "Error : Failed to copy the body index buffer in frameToMesh." << std::endl;
			}
		}

		const UINT16 minZ = pCompositeFrame->m_depthFrame.GetMinDepth();
		const UINT16 maxZ = pCompositeFrame->m_depthFrame.GetMaxDepth();

		std::map<UINT, int> indexMap;
		int nVertexIndex = 0;
		for (UINT h = 0; h < (nFrameHeight - 1); ++h)
		{
			for (UINT w = 0; w < (nFrameWidth - 1); ++w)
			{
				UINT idx1 = w + h * nFrameWidth;
				UINT idx2 = w + 1 + h * nFrameWidth;
				UINT idx3 = w + (h + 1) * nFrameWidth;
				UINT idx4 = w + 1 + (h + 1) * nFrameWidth;

				float rPtZ2 = points[idx2][2];
				bool bIsIdx2Valid = (rPtZ2 > 0.0f)&& depths  && (depths[idx2] > minZ) && (depths[idx2] < maxZ);
				if(!bIsIdx2Valid)
					continue;
				if(bodyIndices && bOnlyShowBody && (0xff == bodyIndices[idx2]))
					continue;

				float rPtZ3 = points[idx3][2];
				bool bIsIdx3Valid = (rPtZ3 > 0.0f) && depths && (depths[idx3] > minZ) && (depths[idx3] < maxZ);
				if(!bIsIdx3Valid)
					continue;
				if(bodyIndices && bOnlyShowBody && (0xff == bodyIndices[idx3]))
					continue;

				float rPtZ1 = points[idx1][2];
				bool bIsIdx1Valid = (rPtZ1 > 0.0f) && depths && (depths[idx1] > minZ) && (depths[idx1] < maxZ);
				bool bBodyMask = bodyIndices && bOnlyShowBody && (0xff == bodyIndices[idx1]);
				if(bIsIdx1Valid && !bBodyMask)
				{
					// Build the upper triangle
					if (abs(rPtZ1 - rPtZ2) < depthThreshold &&
						abs(rPtZ1 - rPtZ3) < depthThreshold &&
						abs(rPtZ2 - rPtZ3) < depthThreshold)
					{
						// The first vertex
						std::map<UINT, int>::iterator itr = indexMap.find(idx1);
						if(itr == indexMap.end())
						{
							SgVec3f color(0.0f, 0.0f, 0.0f);
							SgVec2f uv(uvs[idx1][0] + 0.5f, uvs[idx1][1] + 0.5f);
							if(uv[0] >= 0.0f && uv[0] <= nColorWidth && uv[1] >= 0.0f && uv[1] <= nColorHeight)
							{
								if(pColors)
								{
									BGRQUAD* rgb = pColors + idx1;
									color.setValue((float)(rgb->rgbRed)/255.0f, (float)(rgb->rgbGreen)/255.0f, (float)(rgb->rgbBlue)/255.0f);
								}
								uv.setValue(uv[0]/nColorWidth, 1.0f - uv[1]/nColorHeight);
							}
							else
								uv.setValue(0.0f, 0.0f);
							pMesh->appendPoint(points[idx1], uv, color);
							pMesh->appendTriangleIndex(nVertexIndex);

							indexMap.insert(std::pair<UINT, int>(idx1, nVertexIndex));
							nVertexIndex ++ ;
						}
						else
						{
							pMesh->appendTriangleIndex(itr->second);
						}
						// The second vertex
						itr = indexMap.find(idx2);
						if(itr == indexMap.end())
						{
							SgVec3f color(0.0f, 0.0f, 0.0f);
							SgVec2f uv(uvs[idx2][0] + 0.5f, uvs[idx2][1] + 0.5f);
							if(uv[0] >= 0.0f && uv[0] <= nColorWidth && uv[1] >= 0.0f && uv[1] <= nColorHeight)
							{
								if(pColors)
								{
									BGRQUAD* rgb = pColors + idx2;
									color.setValue((float)(rgb->rgbRed)/255.0f, (float)(rgb->rgbGreen)/255.0f, (float)(rgb->rgbBlue)/255.0f);
								}
								uv.setValue(uv[0]/nColorWidth, 1.0f - uv[1]/nColorHeight);
							}
							else
								uv.setValue(0.0f, 0.0f);
							pMesh->appendPoint(points[idx2], uv, color);
							pMesh->appendTriangleIndex(nVertexIndex);

							indexMap.insert(std::pair<UINT, int>(idx2, nVertexIndex));
							nVertexIndex ++ ;
						}
						else
						{
							pMesh->appendTriangleIndex(itr->second);
						}
						// The third vertex
						itr = indexMap.find(idx3);
						if(itr == indexMap.end())
						{
							SgVec3f color(0.0f, 0.0f, 0.0f);
							SgVec2f uv(uvs[idx3][0] + 0.5f, uvs[idx3][1] + 0.5f);
							if(uv[0] >= 0.0f && uv[0] <= nColorWidth && uv[1] >= 0.0f && uv[1] <= nColorHeight)
							{
								if(pColors)
								{
									BGRQUAD* rgb = pColors + idx3;
									color.setValue((float)(rgb->rgbRed)/255.0f, (float)(rgb->rgbGreen)/255.0f, (float)(rgb->rgbBlue)/255.0f);
								}
								uv.setValue(uv[0]/nColorWidth, 1.0f - uv[1]/nColorHeight);
							}
							else
								uv.setValue(0.0f, 0.0f);
							pMesh->appendPoint(points[idx3], uv, color);
							pMesh->appendTriangleIndex(nVertexIndex);

							indexMap.insert(std::pair<UINT, int>(idx3, nVertexIndex));
							nVertexIndex ++ ;
						}
						else
						{
							pMesh->appendTriangleIndex(itr->second);
						}
					}
				}

				float rPtZ4 = points[idx4][2];
				bool bIsIdx4Valid = (rPtZ4 > 0) && depths && (depths[idx4] > minZ) && (depths[idx4] < maxZ);
				bBodyMask = bodyIndices && bOnlyShowBody && (0xff == bodyIndices[idx4]);
				if(bIsIdx4Valid && !bBodyMask)
				{
					// Build the down triangle
					if (abs(rPtZ4 - rPtZ2) < depthThreshold &&
						abs(rPtZ4 - rPtZ3) < depthThreshold &&
						abs(rPtZ2 - rPtZ3) < depthThreshold)
					{
						// The second vertex
						std::map<UINT, int>::iterator itr = indexMap.find(idx2);
						if(itr == indexMap.end())
						{
							SgVec3f color(0.0f, 0.0f, 0.0f);
							SgVec2f uv(uvs[idx2][0] + 0.5f, uvs[idx2][1] + 0.5f);
							if(uv[0] >= 0.0f && uv[0] <= nColorWidth && uv[1] >= 0.0f && uv[1] <= nColorHeight)
							{
								if(pColors)
								{
									BGRQUAD* rgb = pColors + idx2;
									color.setValue((float)(rgb->rgbRed)/255.0f, (float)(rgb->rgbGreen)/255.0f, (float)(rgb->rgbBlue)/255.0f);
								}
								uv.setValue(uv[0]/nColorWidth, 1.0f - uv[1]/nColorHeight);
							}
							else
								uv.setValue(0.0f, 0.0f);
							pMesh->appendPoint(points[idx2], uv, color);
							pMesh->appendTriangleIndex(nVertexIndex);

							indexMap.insert(std::pair<UINT, int>(idx2, nVertexIndex));
							nVertexIndex ++ ;
						}
						else
						{
							pMesh->appendTriangleIndex(itr->second);
						}
						// The forth vertex
						itr = indexMap.find(idx4);
						if(itr == indexMap.end())
						{
							SgVec3f color(0.0f, 0.0f, 0.0f);
							SgVec2f uv(uvs[idx4][0] + 0.5f, uvs[idx4][1] + 0.5f);
							if(uv[0] >= 0.0f && uv[0] <= nColorWidth && uv[1] >= 0.0f && uv[1] <= nColorHeight)
							{
								if(pColors)
								{
									BGRQUAD* rgb = pColors + idx4;
									color.setValue((float)(rgb->rgbRed)/255.0f, (float)(rgb->rgbGreen)/255.0f, (float)(rgb->rgbBlue)/255.0f);
								}
								uv.setValue(uv[0]/nColorWidth, 1.0f - uv[1]/nColorHeight);
							}
							else
								uv.setValue(0.0f, 0.0f);
							pMesh->appendPoint(points[idx4], uv, color);
							pMesh->appendTriangleIndex(nVertexIndex);

							indexMap.insert(std::pair<UINT, int>(idx4, nVertexIndex));
							nVertexIndex ++ ;
						}
						else
						{
							pMesh->appendTriangleIndex(itr->second);
						}
						// The third vertex
						itr = indexMap.find(idx3);
						if(itr == indexMap.end())
						{
							SgVec3f color(0.0f, 0.0f, 0.0f);
							SgVec2f uv(uvs[idx3][0] + 0.5f, uvs[idx3][1] + 0.5f);
							if(uv[0] >= 0.0f && uv[0] <= nColorWidth && uv[1] >= 0.0f && uv[1] <= nColorHeight)
							{
								if(pColors)
								{
									BGRQUAD* rgb = pColors + idx3;
									color.setValue((float)(rgb->rgbRed)/255.0f, (float)(rgb->rgbGreen)/255.0f, (float)(rgb->rgbBlue)/255.0f);
								}
								uv.setValue(uv[0]/nColorWidth, 1.0f - uv[1]/nColorHeight);
							}
							else
								uv.setValue(0.0f, 0.0f);
							pMesh->appendPoint(points[idx3], uv, color);
							pMesh->appendTriangleIndex(nVertexIndex);

							indexMap.insert(std::pair<UINT, int>(idx3, nVertexIndex));
							nVertexIndex ++ ;
						}
						else
						{
							pMesh->appendTriangleIndex(itr->second);
						}
					}
				}
			}
		}

		return true;
	}
}


