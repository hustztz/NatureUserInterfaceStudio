#pragma once

#include <maya/MString.h>
#include <maya/MColor.h>
#include <maya/MViewport2Renderer.h>

#include <opencv\cv.h>

//Forwards
class NuiDeviceCacheImpl;

class NuiPreviewerRenderOverride : public MHWRender::MRenderOverride
{
private:
	static const int		sMaxPreviewRender = 2;

	enum
	{
		kPreview = 0,		// Preview target
		kPresentOp,			// Present target
		kOperationCount
	};

	enum {
		kDepthTexture = 0,
		kColorTexture,
		kFusionTexture,
		kTextureCount
	};

	enum {
		kOnePreviewShader = 0,	// To preview targets
		kTwoPreviewShader,	// To preview targets
		kShaderCount
	};
public:

	enum PreviewModeFlag
	{
		EPreview_None = 0,
		EPreview_DepthMode = (0x1 << 0),
		EPreview_ColorMode = (0x1 << 1),
		EPreview_FusionMode = (0x1 << 2),
		EPreview_LAST_BIT = (0x1 << 3)
	};

	static const MString kNuiPreviewerRendererName;

	NuiPreviewerRenderOverride(const MString& name);
	virtual ~NuiPreviewerRenderOverride();

	virtual MHWRender::DrawAPI supportedDrawAPIs() const;
	virtual bool startOperationIterator();
	virtual MHWRender::MRenderOperation * renderOperation();
	virtual bool nextRenderOperation();
	virtual MStatus setup(const MString& destination);
	virtual MStatus cleanup();
	virtual MString uiName() const
	{
		return mUIName;
	}
	MHWRender::MRenderOperation * getOperation( unsigned int i)
	{
		if (i < kOperationCount)
			return mRenderOperations[i];
		return NULL;
	}

	// For interface
	void releaseRender();
	void updatePreviewerMode(PreviewModeFlag flags);
	bool updatePreviewerTexture(NuiDeviceCacheImpl* pCache);
	bool refreshView();

protected:
	MStatus updateRenderOperations();
	MStatus updateShaders(const MHWRender::MRenderer *theRenderer);
	MStatus updateOnePreviewShaders(const MHWRender::MShaderManager* shaderMgr, MHWRender::MTexture* texture);
	MStatus updateTwoPreviewShaders(const MHWRender::MShaderManager* shaderMgr, MHWRender::MTexture* texture1, MHWRender::MTexture* texture2);

	bool updateDepthTexture(NuiDeviceCacheImpl* pCache);
	bool updateColorTexture(NuiDeviceCacheImpl* pCache);

	MString mUIName;
	MColor mClearColor;

	MHWRender::MRenderOperation * mRenderOperations[kOperationCount];
	MString mRenderOperationNames[kOperationCount];
	bool mRenderOperationEnabled[kOperationCount];
	int mCurrentOperation;

	MHWRender::MShaderInstance * mShaderInstances[kShaderCount];

private:
	IplImage* mImages[kTextureCount];
	MHWRender::MTexture* mTextures[kTextureCount];
	bool mTextureEnabled[kTextureCount];

	MString mModelPanel;
};

////////////////////////////////////////////////////////////////////////////
// Target preview render
////////////////////////////////////////////////////////////////////////////
class PreviewTargetsOperation : public MHWRender::MQuadRender
{
public:
	PreviewTargetsOperation(const MString &name, NuiPreviewerRenderOverride* theOverride);
	~PreviewTargetsOperation();

	virtual const MHWRender::MShaderInstance * shader();
	virtual const MFloatPoint * viewportRectangleOverride();
	virtual MHWRender::MClearOperation & clearOperation();

	void setShader( MHWRender::MShaderInstance *shader)
	{
		mShaderInstance = shader;
	}
	void setTexture( MHWRender::MTexture *texture )
	{
		mTexture = texture;
	}
	void setRectangle( const MFloatPoint& rec)
	{
		mViewRectangle = rec;
	}

protected:
	NuiPreviewerRenderOverride *mOverride;

	// Shader and texture used for quad render
	MHWRender::MShaderInstance *mShaderInstance;
	MHWRender::MTexture *mTexture;
	MFloatPoint mViewRectangle;
};