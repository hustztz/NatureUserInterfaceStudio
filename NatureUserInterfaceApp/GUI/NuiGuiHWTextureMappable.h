#pragma once

#include "OpenCLUtilities/NuiTextureMappable.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"

#include <pangolin/pangolin.h>

class NuiGuiTextureMappableImpl : public NuiTextureMappableImpl
{
public:
	NuiGuiTextureMappableImpl()
		: _clBuffer(NULL)
	{
	}

	NuiGuiTextureMappableImpl(void* buffer, int width, int height)
		: _clBuffer(NULL)
	{
		update(buffer, width, height);
	}

	~ NuiGuiTextureMappableImpl()
	{
		relaxToCPU();
	}

	virtual void update(const void* buffer, int width, int height) override
	{
		if(width != _rgbTex.width || height != _rgbTex.height)
			_rgbTex.Reinitialise(width, height);

		if(width != _rgbImg.w || height != _rgbImg.h)
		{
			pangolin::FreeImage(_rgbImg);
			_rgbImg.Alloc(width, height, pangolin::VideoFormatFromString("RGBA"));
		}

		if(_rgbImg.ptr && buffer && width*height > 0)
		{
			memcpy(_rgbImg.ptr, buffer, width*height * sizeof(BGRQUAD));
			_rgbTex.Upload(_rgbImg.ptr, GL_BGRA, GL_UNSIGNED_BYTE);
		}
	}

	virtual int width() const override
	{
		return _rgbTex.width;
	}

	virtual int height() const override
	{
		return _rgbTex.height;
	}

	virtual void* data() const override
	{
		return _rgbImg.ptr;
	}

	virtual NuiTextureMappableImpl* clone() const override
	{
		return new NuiGuiTextureMappableImpl(_rgbImg.ptr, (int)_rgbImg.w, (int)_rgbImg.h);
	}

	GLuint data()
	{
		return _rgbTex.IsValid() ? _rgbTex.tid : 0;
	}

	void render()
	{
		_rgbTex.RenderToViewport(true);
	}

	cl_mem clBuffer()
	{
		if(!_rgbTex.IsValid())
			return NULL;

		//NuiOpenGLThread::instance().enqueueAndWait([&]()
		//{
		if (!NuiGPUMemManager::instance().isValidCLBuffer(_clBuffer))
		{
			GLuint textureId[1] = {_rgbTex.tid};
			_clBuffer = NuiGPUMemManager::instance().CreateCLTextureFromHWBuffer(
				CL_MEM_READ_WRITE, textureId,
				NULL, NuiGPUMemSharedType::XG_GPUMEM_SHARED_Tex,
				"");
			assert(_clBuffer);
		}
		//});
		return _clBuffer;
	}

	void relaxToCPU() override
	{
		//NuiOpenGLThread::instance().enqueueAndWait([&]()
		//{
		if (NuiGPUMemManager::instance().isValidCLBuffer(_clBuffer)) {
			// Delete the OpenCL shared object
			cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(_clBuffer);
			NUI_CHECK_CL_ERR(err);
			_clBuffer = nullptr;
		}

		// Realize to system memory
		_rgbTex.Delete();
		pangolin::FreeImage(_rgbImg);
		//});
	}
private:
	pangolin::GlTexture		_rgbTex;
	pangolin::TypedImage	_rgbImg;
	cl_mem					_clBuffer;
};

struct NuiGuiHWTextureMappable
{
	static GLuint asHWTextureBuffer(NuiTextureMappable& mappable)
	{
		// Get the underlying buffer impl
		std::shared_ptr<NuiGuiTextureMappableImpl> hwImpl =
			NuiTextureMappableAccessor::asImpl<NuiGuiTextureMappableImpl>(mappable);
		if (hwImpl)
			return hwImpl->data();

		// Copy to hardware buffer
		hwImpl.reset(new NuiGuiTextureMappableImpl(mappable.data(), mappable.width(), mappable.height()));

		// Change to hardware buffer in-place
		 NuiTextureMappableAccessor::setImpl(mappable, hwImpl);
		return hwImpl->data();
	}

	static void renderHWTextureBuffer(NuiTextureMappable& mappable)
	{
		// Get the underlying buffer impl
		std::shared_ptr<NuiGuiTextureMappableImpl> hwImpl =
			NuiTextureMappableAccessor::asImpl<NuiGuiTextureMappableImpl>(mappable);
		if (!hwImpl)
		{
			// Copy to hardware buffer
			hwImpl.reset(new NuiGuiTextureMappableImpl(mappable.data(), mappable.width(), mappable.height()));

			// Change to hardware buffer in-place
			NuiTextureMappableAccessor::setImpl(mappable, hwImpl);
		}

		hwImpl->render();
	}

	static cl_mem asHWTextureBufferSharedWithCL(NuiTextureMappable& mappable)
	{
		// Convert to hw texture buffer
		asHWTextureBuffer(mappable);

		// Create OpenCL wrapper object
		std::shared_ptr<NuiGuiTextureMappableImpl> hwImpl =
			NuiTextureMappableAccessor::asImpl<NuiGuiTextureMappableImpl>(mappable);
		return hwImpl ? hwImpl->clBuffer() : NULL;
	}
};