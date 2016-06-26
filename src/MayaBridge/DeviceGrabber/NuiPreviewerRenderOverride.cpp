#include "NuiPreviewerRenderOverride.h"

#include <maya/MShaderManager.h>
#include <maya/MDrawContext.h>
#include <maya/MAnimControl.h>
#include <maya/M3dView.h>
#include <maya/MTextureManager.h>
#include <maya/MGlobal.h>

#include "DeviceManager/DeviceCache/NuiDeviceCacheImpl.h"
#include "DeviceManager/DeviceCache/NuiCompositeFrame.h"

using namespace cv;

const MString NuiPreviewerRenderOverride::kNuiPreviewerRendererName = "GrabberPreviewer";

NuiPreviewerRenderOverride::NuiPreviewerRenderOverride(const MString& name)
	: MRenderOverride(name)
	, mUIName("Device Grabber Previewer")
{
	unsigned int i = 0;
	for (i=0; i<kOperationCount; i++)
	{
		mRenderOperations[i] = NULL;
	}
	mCurrentOperation = -1;

	for (i=0; i<kShaderCount; i++)
	{
		mShaderInstances[i] = NULL;
	}

	for (i=0; i<kTextureCount; i++)
	{
		mTextures[i] = NULL;
		mImages[i] = NULL;
		mTextureEnabled[i] = false;
	}
}

NuiPreviewerRenderOverride::~NuiPreviewerRenderOverride()
{
	releaseRender();
}

void NuiPreviewerRenderOverride::releaseRender()
{
	for (unsigned int i=0; i<kOperationCount; i++)
	{
		delete mRenderOperations[i];
		mRenderOperations[i] = NULL;
	}

	MHWRender::MRenderer* theRenderer = MHWRender::MRenderer::theRenderer();
	if (theRenderer)
	{
		// Release shaders
		const MHWRender::MShaderManager* shaderMgr = theRenderer->getShaderManager();
		for (unsigned int i=0; i<kShaderCount; i++)
		{
			if (mShaderInstances[i])
			{
				if (shaderMgr)
					shaderMgr->releaseShader(mShaderInstances[i]);
				mShaderInstances[i] = NULL;
			}
		}

		// Release textures
		const MHWRender::MTextureManager* textureManager = theRenderer->getTextureManager();
		for (unsigned int i=0; i<kTextureCount; i++)
		{
			if (mTextures[i])
			{
				if (textureManager)
				{
					textureManager->releaseTexture(mTextures[i]);
					mTextures[i] = NULL;
				}
			}
			if (mImages[i])
			{
				cvReleaseImage(&mImages[i]);
				mImages[i] = NULL;
			}
		}
	}
}

MHWRender::DrawAPI NuiPreviewerRenderOverride::supportedDrawAPIs() const
{
	return MHWRender::kAllDevices;
}

bool NuiPreviewerRenderOverride::startOperationIterator()
{
	mCurrentOperation = 0;
	return true;
}

MHWRender::MRenderOperation* NuiPreviewerRenderOverride::renderOperation()
{
	if (mCurrentOperation >= 0 && mCurrentOperation < kOperationCount)
	{
		// Skip empty and disabled operations
		//
		while(!mRenderOperations[mCurrentOperation] || !mRenderOperationEnabled[mCurrentOperation])
		{
			mCurrentOperation++;
			if (mCurrentOperation >= kOperationCount)
			{
				return NULL;
			}
		}

		if (mRenderOperations[mCurrentOperation])
		{
			return mRenderOperations[mCurrentOperation];
		}
	}
	return NULL;
}

bool NuiPreviewerRenderOverride::nextRenderOperation()
{
	mCurrentOperation++;
	/*if(mCurrentOperation == mPreviewTargetCount)
	{
		unsigned int disableOperationCount = 4 - mPreviewTargetCount;
		mCurrentOperation += disableOperationCount;
	}*/
	return (mCurrentOperation < kOperationCount);
}

//
// Update list of operations to perform:
//
//
//		1. Play the cached target.
//		2. Blit on-screen
//
MStatus NuiPreviewerRenderOverride::updateRenderOperations()
{
	bool initOperations = true;
	for (unsigned int i=0; i<kOperationCount; i++)
	{
		if (mRenderOperations[i])
			initOperations = false;
	}

	if (initOperations)
	{
		// There ops are for player
		// A quad blit to preview a target
		mRenderOperationNames[kPreview] = "_NuiGrabberPreviewer_TargetPreview";
		PreviewTargetsOperation * previewOp = new PreviewTargetsOperation (mRenderOperationNames[kPreview], this);
		mRenderOperations[kPreview] = previewOp;
		mRenderOperationEnabled[kPreview] = true;

		// Generic screen blit - always want to do this
		mRenderOperationNames[kPresentOp] = "_NuiGrabberPreviewer_PresentTarget";
		mRenderOperations[kPresentOp] = new MHWRender::MPresentTarget(mRenderOperationNames[kPresentOp]);
		mRenderOperationEnabled[kPresentOp] = true;
	}
	mCurrentOperation = -1;

	MStatus haveOperations = MStatus::kFailure;
	for (unsigned int i=0; i<kOperationCount; i++)
	{
		if (mRenderOperations[i])
		{
			haveOperations = MStatus::kSuccess;
		}
	}

	return haveOperations;
}

void NuiPreviewerRenderOverride::updatePreviewerMode(PreviewModeFlag flags)
{
	int previewerCount = 0;
	if(flags & EPreview_DepthMode)
	{
		mTextureEnabled[kDepthTexture] = true;
		previewerCount ++;
	}
	else
	{
		mTextureEnabled[kDepthTexture] = false;
	}
	if(flags & EPreview_ColorMode)
	{
		mTextureEnabled[kColorTexture] = true;
		previewerCount ++;
	}
	else
	{
		mTextureEnabled[kColorTexture] = false;
	}
	if((flags & EPreview_FusionMode) && (previewerCount < sMaxPreviewRender))
	{
		mTextureEnabled[kFusionTexture] = true;
		previewerCount ++;
	}
	else
	{
		mTextureEnabled[kFusionTexture] = false;
	}
}

//
// Update all targets used for rendering
//
MStatus NuiPreviewerRenderOverride::updateShaders(const MHWRender::MRenderer *theRenderer)
{
	if (!theRenderer)
		return MStatus::kFailure;

	const MHWRender::MShaderManager* shaderMgr = theRenderer->getShaderManager();
	if (!shaderMgr) return MStatus::kFailure;

	MHWRender::MTexture* previewTextures[kTextureCount];
	int previewerCount = 0;
	for (int i=0; i<kTextureCount; i++)
	{
		if (mTextureEnabled[i])
		{
			previewTextures[previewerCount] = mTextures[i];
			previewerCount ++;
		}
	}
	assert(previewerCount <= sMaxPreviewRender);
	if(0 == previewerCount)
	{
		return MStatus::kFailure;
	}
	
	switch (previewerCount)
	{
	case 1:
		return updateOnePreviewShaders(shaderMgr, previewTextures[0]);
	case 2:
		return updateTwoPreviewShaders(shaderMgr, previewTextures[0], previewTextures[1]);
	default:
		return MStatus::kFailure;
	}

	return MStatus::kSuccess;
}

//
// Update all shaders used for rendering
//
MStatus NuiPreviewerRenderOverride::updateOnePreviewShaders(const MHWRender::MShaderManager* shaderMgr, MHWRender::MTexture* texture)
{
	if (!shaderMgr) return MStatus::kFailure;
	// Set up a preview target shader (Targets as input)
	//
	MHWRender::MShaderInstance *shaderInstance = mShaderInstances[kOnePreviewShader];
	if (!shaderInstance)
	{
		shaderInstance = shaderMgr->getEffectsFileShader( "Copy", "" );
		mShaderInstances[kOnePreviewShader] = shaderInstance;

		// Set constant parameters
		if (shaderInstance)
		{
			// We want to make sure to reblit back alpha as well as RGB
			shaderInstance->setParameter("gDisableAlpha", false );
			shaderInstance->setParameter("gVerticalFlip", false );
		}
	}
	MFloatPoint scaleSize( 0.0f, 0.0f, 1.0f,  1.0f );
	// Make sure to update the texture to use
	MHWRender::MTextureAssignment texAssignment;
	texAssignment.texture = texture;
	// Update the parameter for the texture assignment.
	shaderInstance->setParameter("gInputTex", texAssignment );
	// Update the resize of texture on the viewport
	if(texAssignment.texture)
	{
		MHWRender::MTextureDescription textureDesc;
		texAssignment.texture->textureDescription(textureDesc);
		unsigned int targetWidth = 0;
		unsigned int targetHeight = 0;
		MHWRender::MRenderer *theRenderer = MHWRender::MRenderer::theRenderer();
		if( theRenderer )
		{
			// Update the scale of width and height in the shader.
			// The width and height of the preview texture should be in keep with the current viewport.
			theRenderer->outputTargetSize( targetWidth, targetHeight );
			float scaleY = (0 == targetHeight) ? 1.0f : (float)textureDesc.fHeight*targetWidth/textureDesc.fWidth/targetHeight;
			if(scaleY < 1.0f)
			{
				scaleSize[1] = (1.0f-scaleY)/2;
				scaleSize[3] = scaleY;
			}
		}
	}

	// Update shader and texture on quad operation
	PreviewTargetsOperation * quadOp = (PreviewTargetsOperation * )mRenderOperations[kPreview];
	if (quadOp)
	{
		quadOp->setShader( mShaderInstances[kOnePreviewShader] );
		quadOp->setRectangle( scaleSize );
	}

	if (quadOp && shaderInstance)
		return MStatus::kSuccess;
	return MStatus::kFailure;
}

MStatus NuiPreviewerRenderOverride::updateTwoPreviewShaders(const MHWRender::MShaderManager* shaderMgr, MHWRender::MTexture* texture1, MHWRender::MTexture* texture2)
{
	MHWRender::MShaderInstance *shaderInstance = mShaderInstances[kTwoPreviewShader];
	if (!shaderInstance)
	{
		shaderInstance = shaderMgr->getEffectsFileShader( "FreeView", "" );
		mShaderInstances[kTwoPreviewShader] = shaderInstance;

		// Set constant parmaeters
		if (shaderInstance)
		{
			const float borderClr[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
			const float backGroundClr[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
			shaderInstance->setParameter("gBorderColor", borderClr );
			shaderInstance->setParameter("gBackgroundColor", backGroundClr );
		}
	}
	// Update shader's per frame parameters
	if (shaderInstance)
	{
		unsigned int targetWidth = 0;
		unsigned int targetHeight = 0;
		MHWRender::MRenderer *theRenderer = MHWRender::MRenderer::theRenderer();
		if( theRenderer )
			theRenderer->outputTargetSize( targetWidth, targetHeight );

		float vpSize[2] = { (float)targetWidth,  (float)targetHeight };
		shaderInstance->setParameter("gViewportSizePixels", vpSize );

		float sourceSize[2] = { (float)targetWidth,  (float)targetHeight };
		shaderInstance->setParameter("gSourceSizePixels", sourceSize );

		/// Could use 0.0125 * width / 2
		shaderInstance->setParameter("gBorderSizePixels", 0.00625f * targetWidth );

		// Bind two input targets
		if (texture1)
		{
			MHWRender::MTextureAssignment texAssignment;
			texAssignment.texture = texture1;
			shaderInstance->setParameter("gSourceTex", texAssignment);
		}
		if (texture2)
		{
			MHWRender::MTextureAssignment assignment2;
			assignment2.texture = texture2;
			shaderInstance->setParameter("gSourceTex2", assignment2);
		}
	}
	// Update shader on quad operation
	PreviewTargetsOperation * quadOp = (PreviewTargetsOperation * )mRenderOperations[kPreview];
	if (quadOp)
		quadOp->setShader( mShaderInstances[kTwoPreviewShader] );

	if (quadOp && shaderInstance)
		return MStatus::kSuccess;
	return MStatus::kFailure;
}

//
// Update override for the current frame
//
MStatus NuiPreviewerRenderOverride::setup(const MString& destination)
{
	// Firewall checks
	MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
	if (!renderer) return MStatus::kFailure;

	// Update render operations
	MStatus status = updateRenderOperations();
	if (status != MStatus::kSuccess)
		return status;

	// Update shaders
	status = updateShaders( renderer );

	return status;
}

MStatus NuiPreviewerRenderOverride::cleanup()
{
	mCurrentOperation = -1;
	return MStatus::kSuccess;
}

bool NuiPreviewerRenderOverride::updatePreviewerTexture(NuiDeviceCacheImpl* pCache)
{
	assert(pCache);
	if(!pCache)
		return false;

	bool hasUpdated = false;

	if(mTextureEnabled[kDepthTexture])
	{
		hasUpdated |= updateDepthTexture(pCache);
	}
	if(mTextureEnabled[kColorTexture])
	{
		hasUpdated |= updateColorTexture(pCache);
	}
	
	return hasUpdated;
}

bool NuiPreviewerRenderOverride::updateDepthTexture(NuiDeviceCacheImpl* pCache)
{
	assert(pCache);
	if(!pCache)
		return false;

	if(!mTextureEnabled[kDepthTexture])
		return false;

	NuiCompositeFrame* pCompositeFrame = pCache->GetLatestCompoundFrame();
	if (!pCompositeFrame)
		return false;

	const NuiDepthFrame* pDepthFrame = pCompositeFrame->GetDepthFrame();
	if(!pDepthFrame)
		return false;

	UINT nDepthBufferSize = pDepthFrame->GetDepthBufferSize();
	UINT16* pDepthBuffer = pDepthFrame->GetDepthBuffer();
	if(!pDepthBuffer || nDepthBufferSize == 0)
		return false;

	UINT nDepthWidth = pDepthFrame->GetDepthWidth();
	UINT nDepthHeight = pDepthFrame->GetDepthHeight();
	if(mImages[kDepthTexture])
	{
		if(nDepthWidth != mImages[kDepthTexture]->width || nDepthHeight != mImages[kDepthTexture]->height)
		{
			cvReleaseImage(&mImages[kDepthTexture]);
			mImages[kDepthTexture] = NULL;
		}
	}
	if(!mImages[kDepthTexture])
	{
		mImages[kDepthTexture] = cvCreateImage(cvSize(nDepthWidth,nDepthHeight),IPL_DEPTH_16U,1);
	}

	const unsigned int cBytePerPixel = 2;
	if(mImages[kDepthTexture]->imageData && pDepthBuffer)
	{
		pCompositeFrame->ReadDepthLock();
		errno_t err = memcpy_s(mImages[kDepthTexture]->imageData, mImages[kDepthTexture]->imageSize * cBytePerPixel, reinterpret_cast<BYTE*>(pDepthBuffer), nDepthBufferSize);
		pCompositeFrame->ReadDepthUnlock();
		if(0 != err)
		{
			printf_s( "Failed to read cached images in previewer.\n" );
			return false;
		}
	}
	else
	{
		printf_s( "Failed to read cached images in previewer.\n" );
		return false;
	}

	MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
	if(!renderer)
		return false;
	MHWRender::MTextureManager* textureManager = renderer->getTextureManager();
	if(!textureManager)
	{
		//MGlobal::displayWarning( hwApiTextureTestStrings::getString( hwApiTextureTestStrings::kErrorTextureManager ) );
		return false;
	}

	
	MString texName = MString("grabberDepthImage");
	mTextures[kDepthTexture] = textureManager->findTexture(texName);
	if(mTextures[kDepthTexture])
	{
		mTextures[kDepthTexture]->update( mImages[kDepthTexture]->imageData, false, cBytePerPixel*nDepthWidth );
	}
	else
	{
		MHWRender::MTextureDescription textureDesc;
		textureDesc.fWidth = nDepthWidth;
		textureDesc.fHeight = nDepthHeight;
		textureDesc.fDepth = 1;
		textureDesc.fBytesPerRow = cBytePerPixel*nDepthWidth;
		textureDesc.fBytesPerSlice = cBytePerPixel*nDepthWidth*nDepthHeight;
		textureDesc.fMipmaps = 1;
		textureDesc.fArraySlices = 1;
		textureDesc.fFormat = MHWRender::kR16_UNORM;
		textureDesc.fTextureType = MHWRender::kImage2D;
		textureDesc.fEnvMapType = MHWRender::kEnvNone;

		// Construct the texture with the screen pixels
		mTextures[kDepthTexture] = textureManager->acquireTexture( texName, textureDesc, mImages[kDepthTexture]->imageData, false );
	}
	if(!mTextures[kDepthTexture])
	{
		printf_s("Preview texture is NULL!\n");
		return false;
	}
	
	return true;
}

bool NuiPreviewerRenderOverride::updateColorTexture(NuiDeviceCacheImpl* pCache)
{
	assert(pCache);
	if(!pCache)
		return false;

	if(!mTextureEnabled[kColorTexture])
		return false;

	NuiCompositeFrame* pCompositeFrame = pCache->GetLatestCompoundFrame();
	if (!pCompositeFrame)
		return false;

	const NuiColorFrame* pColorFrame = pCompositeFrame->GetColorFrame();
	assert(pColorFrame);
	if(!pColorFrame)
		return false;

	BYTE* pColorBuffer = reinterpret_cast<BYTE*>(pColorFrame->GetColorBuffer());
	UINT nColorBufferSize = pColorFrame->GetColorBufferSize();
	if(!pColorBuffer || nColorBufferSize == 0)
		return false;

	UINT nColorWidth = pColorFrame->GetColorWidth();
	UINT nColorHeight = pColorFrame->GetColorHeight();	
	if(mImages[kColorTexture])
	{
		if(nColorWidth != mImages[kColorTexture]->width || nColorHeight != mImages[kColorTexture]->height)
		{
			cvReleaseImage(&mImages[kColorTexture]);
			mImages[kColorTexture] = NULL;
		}
	}
	if(!mImages[kColorTexture])
	{
		mImages[kColorTexture] = cvCreateImage(cvSize(nColorWidth,nColorHeight),IPL_DEPTH_32S,1);
	}

	const unsigned int cBytePerPixel = 4;
	if(mImages[kColorTexture]->imageData)
	{
		pCompositeFrame->ReadColorLock();
		errno_t err = memcpy_s(mImages[kColorTexture]->imageData, mImages[kColorTexture]->imageSize * cBytePerPixel, pColorBuffer, nColorBufferSize);
		pCompositeFrame->ReadColorUnlock();
		if(0 != err)
		{
			printf_s( "Failed to read cached images in previewer.\n" );
			return false;
		}
	}
	else
	{
		printf_s( "Failed to read cached images in previewer.\n" );
		return false;
	}

	MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
	if(!renderer)
		return false;
	MHWRender::MTextureManager* textureManager = renderer->getTextureManager();
	if(!textureManager)
	{
		//MGlobal::displayWarning( hwApiTextureTestStrings::getString( hwApiTextureTestStrings::kErrorTextureManager ) );
		return false;
	}
	
	MString texName = MString("grabberColorImage");
	mTextures[kColorTexture] = textureManager->findTexture(texName);
	if(mTextures[kColorTexture])
	{
		mTextures[kColorTexture]->update( mImages[kColorTexture]->imageData, false, cBytePerPixel*nColorWidth );
	}
	else
	{
		MHWRender::MTextureDescription textureDesc;
		textureDesc.fWidth = nColorWidth;
		textureDesc.fHeight = nColorHeight;
		textureDesc.fDepth = 1;
		textureDesc.fBytesPerRow = cBytePerPixel*nColorWidth;
		textureDesc.fBytesPerSlice = cBytePerPixel*nColorWidth*nColorHeight;
		textureDesc.fMipmaps = 1;
		textureDesc.fArraySlices = 1;
		textureDesc.fFormat = MHWRender::kB8G8R8A8;
		textureDesc.fTextureType = MHWRender::kImage2D;
		textureDesc.fEnvMapType = MHWRender::kEnvNone;

		// Construct the texture with the screen pixels
		mTextures[kColorTexture] = textureManager->acquireTexture( texName, textureDesc, mImages[kColorTexture]->imageData, false );
	}
	if(!mTextures[kColorTexture])
	{
		printf_s("Preview texture is NULL!\n");
		return false;
	}
	return true;
}

bool NuiPreviewerRenderOverride::refreshView()
{
	M3dView view;
	if (M3dView::getM3dViewFromModelPanel(mModelPanel, view) != MStatus::kSuccess)
		return false;

	view.scheduleRefresh();
	return true;
}

///////////////////////////////////////////////////////////////////

PreviewTargetsOperation::PreviewTargetsOperation(const MString &name, NuiPreviewerRenderOverride *theOverride)
	: MQuadRender( name )
	, mShaderInstance(NULL)
	, mTexture(NULL)
	, mOverride(theOverride)
{
	// 100 % of target size
	mViewRectangle[0] = 0.0f;
	mViewRectangle[1] = 0.0f;
	mViewRectangle[2] = 1.0f;
	mViewRectangle[3] = 1.0f;
}

PreviewTargetsOperation::~PreviewTargetsOperation()
{
	mShaderInstance = NULL;
	mTexture = NULL;
	mOverride = NULL;
}

const MHWRender::MShaderInstance* PreviewTargetsOperation::shader()
{
	return mShaderInstance;
}

const MFloatPoint * PreviewTargetsOperation::viewportRectangleOverride()
{
	return &mViewRectangle;
}

MHWRender::MClearOperation &
	PreviewTargetsOperation::clearOperation()
{
	mClearOperation.setMask( (unsigned int) MHWRender::MClearOperation::kClearAll );
	return mClearOperation;
}