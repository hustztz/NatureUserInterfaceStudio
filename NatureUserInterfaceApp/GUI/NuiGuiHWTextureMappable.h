#include "stdafx.h"
#include "OpenCLUtilities/NuiTextureMappable.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"

#include <pangolin/gl/glsl.h>
#include <pangolin/pangolin.h>

class NuiGuiTextureMappableImpl : public NuiTextureMappableImpl
{
public:
	NuiGuiTextureMappableImpl()
		: _clBuffer(NULL)
	{
		_rgbTex.Delete();
		pangolin::FreeImage(_rgbImg);
	}

	NuiGuiTextureMappableImpl(void* buffer, int width, int height)
		: _clBuffer(NULL)
	{
		if(buffer && width*height > 0)
		{
			if(width != _rgbTex.width || height != _rgbTex.height)
				_rgbTex.Reinitialise(width, height);

			if(width != _rgbImg.w || height != _rgbImg.h)
			{
				pangolin::FreeImage(_rgbImg);
				_rgbImg.Alloc(width, height, pangolin::VideoFormatFromString("RGBA"));
			}

			memcpy(_rgbImg.ptr, buffer, width*height * sizeof(BGRQUAD));

			_rgbTex.Upload(_rgbImg.ptr, GL_BGRA, GL_UNSIGNED_BYTE);
		}
	}

	~ NuiGuiTextureMappableImpl()
	{
		_rgbTex.Delete();
		pangolin::FreeImage(_rgbImg);
		_clBuffer = NULL;
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
		return _rgbTex.ptr;
	}

	virtual NuiTextureMappableImpl* clone() const override
	{
		return new NuiGuiTextureMappableImpl(_rgbImg.ptr, _rgbImg.w, _rgbImg.h);
	}

	GLuint data()
	{
		return _rgbTex.IsValid() ? _rgbTex.tid : 0;
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
			_rgbTex.Delete();
		}

		// Realize to system memory
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
			std::dynamic_pointer_cast<NuiGuiTextureMappableImpl>(mappable._impl);
		if (hwImpl)
			return hwImpl->data();

		// Copy to hardware buffer
		NuiImageMappableBuffer buffer(mappable);
		hwImpl.reset(new NuiGuiTextureMappableImpl(buffer.data(), buffer.width(), buffer.height()));

		// Change to hardware buffer in-place
		mappable._impl = std::static_pointer_cast<NuiTextureMappableImpl>(hwImpl);
		return hwImpl->data();
	}

	static cl_mem asHWTextureBufferSharedWithCL(NuiTextureMappable& mappable)
	{
		if(mappable.width()*mappable.height() == 0)
			return NULL;

		// Convert to hw texture buffer
		asHWTextureBuffer(mappable);

		// Create OpenCL wrapper object
		std::shared_ptr<NuiGuiTextureMappableImpl> hwImpl =
			std::dynamic_pointer_cast<NuiGuiTextureMappableImpl>(mappable._impl);
		return hwImpl ? hwImpl->clBuffer() : NULL;
	}
}