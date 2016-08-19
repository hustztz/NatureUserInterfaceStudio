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
		return (int)_rgbTex.width;
	}

	virtual int height() const override
	{
		return (int)_rgbTex.height;
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

class NuiGuiFrameTextureMappableImpl : public NuiTextureMappableImpl
{
public:
	NuiGuiFrameTextureMappableImpl()
		: _clBuffer(NULL)
	{
	}

	NuiGuiFrameTextureMappableImpl(void* buffer, int width, int height)
		: _clBuffer(NULL)
	{
		update(buffer, width, height);
	}

	~ NuiGuiFrameTextureMappableImpl()
	{
		relaxToCPU();
	}

	virtual void update(const void* buffer, int width, int height) override
	{
		if(width != _rgbTex.width || height != _rgbTex.height)
		{
			_rgbTex.Reinitialise(width, height);
			_frameBuffer.AttachColour(_rgbTex);

			GLenum status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
			if(status != GL_FRAMEBUFFER_COMPLETE_EXT)  
			{  
				switch(status)  
				{  
				case GL_FRAMEBUFFER_COMPLETE_EXT:
					break;
				case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT:
					break;
				default:
					break;  
				}  
			}
		}

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
		return (int)_rgbTex.width;
	}

	virtual int height() const override
	{
		return (int)_rgbTex.height;
	}

	virtual void* data() const override
	{
		return _rgbImg.ptr;
	}

	virtual NuiGuiFrameTextureMappableImpl* clone() const override
	{
		return new NuiGuiFrameTextureMappableImpl(_rgbImg.ptr, (int)_rgbImg.w, (int)_rgbImg.h);
	}

	GLuint data()
	{
		return _rgbTex.IsValid() ? _rgbTex.tid : 0;
	}

	void bind() const
	{
		_frameBuffer.Bind();
	}

	void unbind() const
	{
		_frameBuffer.Unbind();
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
	pangolin::GlFramebuffer _frameBuffer;
	pangolin::TypedImage	_rgbImg;
	cl_mem					_clBuffer;
};

class NuiGuiRenderBufferMappableImpl : public NuiTextureMappableImpl
{
public:
	NuiGuiRenderBufferMappableImpl()
		: _clBuffer(NULL)
	{
		//NuiOpenGLThread::instance().enqueueAndWait([&]()
		//{
		glGenRenderbuffers(1, _rbo); // Generate our Render Buffer Object
		//});
	}

	NuiGuiRenderBufferMappableImpl(const void* buffer, int width, int height)
		: _clBuffer(NULL)
	{
		update(NULL, width, height);
		//NuiOpenGLThread::instance().enqueueAndWait([&]()
		//{
		glGenRenderbuffers(1, _rbo); // Generate our Render Buffer Object
		if(_rgbImg.w*_rgbImg.h > 0)
		{
			//glEnable(GL_RENDERBUFFER);
			glBindRenderbuffer(GL_RENDERBUFFER, _rbo[0]);
			glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA4, width, height);
			glBindRenderbuffer(GL_RENDERBUFFER, 0);
			//glDisable(GL_RENDERBUFFER);
		}
		//});
	}

	virtual ~NuiGuiRenderBufferMappableImpl()
	{
		relaxToCPU();
	}

	virtual void update(const void* buffer, int width, int height) override
	{
		if(width != _rgbImg.w || height != _rgbImg.h)
		{
			pangolin::FreeImage(_rgbImg);
			_rgbImg.Alloc(width, height, pangolin::VideoFormatFromString("RGBA"));
		}
		if(_rgbImg.ptr && buffer && width*height > 0)
		{
			memcpy(_rgbImg.ptr, buffer, width*height * sizeof(BGRQUAD));
		}
	}

	virtual int width() const override
	{
		return (int)_rgbImg.w;
	}

	virtual int height() const override
	{
		return (int)_rgbImg.h;
	}

	virtual void* data() const override
	{
		return _rgbImg.ptr;
	}

	virtual NuiTextureMappableImpl* clone() const override
	{
		return new NuiGuiRenderBufferMappableImpl(_rgbImg.ptr, (int)_rgbImg.w, (int)_rgbImg.h);
	}

	GLuint data()
	{
		return _rbo[0];
	}

	cl_mem clBuffer(const char* name=nullptr)
	{
		//NuiOpenGLThread::instance().enqueueAndWait([&]()
		//{
		if (!NuiGPUMemManager::instance().isValidCLBuffer(_clBuffer))
		{
			_clBuffer = NuiGPUMemManager::instance().CreateCLObjectFromRenderBuffer(
				CL_MEM_READ_WRITE, _rbo,
				NULL, NuiGPUMemSharedType::XG_GPUMEM_SHARED_VB,
				name?name:"");
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
		glDeleteRenderbuffers(1, _rbo);
		//});

		pangolin::FreeImage(_rgbImg);
	}

private:
	GLuint									_rbo[1];
	pangolin::TypedImage					_rgbImg;
	cl_mem                                  _clBuffer;
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

	// Frame texture buffer
	static GLuint asFrameTextureBuffer(NuiTextureMappable& mappable)
	{
		// Get the underlying buffer impl
		std::shared_ptr<NuiGuiFrameTextureMappableImpl> hwImpl =
			NuiTextureMappableAccessor::asImpl<NuiGuiFrameTextureMappableImpl>(mappable);
		if (hwImpl)
			return hwImpl->data();

		// Copy to hardware buffer
		hwImpl.reset(new NuiGuiFrameTextureMappableImpl(mappable.data(), mappable.width(), mappable.height()));

		// Change to hardware buffer in-place
		NuiTextureMappableAccessor::setImpl(mappable, hwImpl);
		return hwImpl->data();
	}

	static cl_mem asFrameTextureBufferSharedWithCL(NuiTextureMappable& mappable)
	{
		// Convert to hw texture buffer
		asFrameTextureBuffer(mappable);

		// Create OpenCL wrapper object
		std::shared_ptr<NuiGuiFrameTextureMappableImpl> hwImpl =
			NuiTextureMappableAccessor::asImpl<NuiGuiFrameTextureMappableImpl>(mappable);
		return hwImpl ? hwImpl->clBuffer() : NULL;
	}

	static void bindFrameTextureBuffer(NuiTextureMappable& mappable)
	{
		// Get the underlying buffer impl
		std::shared_ptr<NuiGuiFrameTextureMappableImpl> hwImpl =
			NuiTextureMappableAccessor::asImpl<NuiGuiFrameTextureMappableImpl>(mappable);
		if (!hwImpl)
		{
			// Copy to hardware buffer
			hwImpl.reset(new NuiGuiFrameTextureMappableImpl(mappable.data(), mappable.width(), mappable.height()));

			// Change to hardware buffer in-place
			NuiTextureMappableAccessor::setImpl(mappable, hwImpl);
		}

		hwImpl->bind();
	}

	static void unbindFrameTextureBuffer(NuiTextureMappable& mappable)
	{
		// Get the underlying buffer impl
		std::shared_ptr<NuiGuiFrameTextureMappableImpl> hwImpl =
			NuiTextureMappableAccessor::asImpl<NuiGuiFrameTextureMappableImpl>(mappable);
		if (hwImpl)
		{
			hwImpl->unbind();
		}
	}

	// Render buffer object
	static GLuint asHWRenderBuffer(NuiTextureMappable& mappable)
	{
		// Get the underlying buffer impl
		std::shared_ptr<NuiGuiRenderBufferMappableImpl > hwImpl =
			NuiTextureMappableAccessor::asImpl<NuiGuiRenderBufferMappableImpl >(mappable);
		if (hwImpl)
			return hwImpl->data();

		hwImpl.reset(new NuiGuiRenderBufferMappableImpl(mappable.data(), mappable.width(), mappable.height()));

		// Make a mappable
		NuiTextureMappableAccessor::setImpl(mappable, hwImpl);
		return hwImpl->data();
	}

	static cl_mem asHWRenderBufferSharedWithCL(NuiTextureMappable& mappable)
	{
		// Convert to index buffer
		asHWRenderBuffer(mappable);

		// Create OpenCL wrapper object
		std::shared_ptr<NuiGuiRenderBufferMappableImpl> hwImpl =
			NuiTextureMappableAccessor::asImpl<NuiGuiRenderBufferMappableImpl>(mappable);
		return hwImpl ? hwImpl->clBuffer() : NULL;
	}
};