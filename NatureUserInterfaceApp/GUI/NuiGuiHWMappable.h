#pragma once

#include "OpenCLUtilities/NuiMappable.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"
#include "OpenCLUtilities/NuiOpenGLThread.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"
#include "Foundation/NuiDebugMacro.h"

#include <pangolin/gl/glsl.h>

class NuiGuiVertexBufferMappableImpl : public NuiMappableImpl
{
public:
	NuiGuiVertexBufferMappableImpl()
		: _mappedBuffer(NULL)
		, _clBuffer(NULL)
		, _num(0)
		, _elementSize(0)
	{}

	NuiGuiVertexBufferMappableImpl(void* buffer, size_t num, size_t elementSize)
		: _clBuffer(NULL)
		, _num(num)
		, _elementSize(elementSize)
	{
		if(_num*_elementSize > 0 && buffer)
		{
			_mappedBuffer = malloc(_num*_elementSize);
			memcpy(_mappedBuffer, buffer, _num*_elementSize);
			//NuiOpenGLThread::instance().enqueueAndWait([&]()
			//{
			glGenBuffers(1, _vbo); // Generate our Vertex Buffer Object
			glBindBuffer(GL_ARRAY_BUFFER, _vbo[0]); // Bind our Vertex Buffer Object
			glBufferData(GL_ARRAY_BUFFER, _num*_elementSize, _mappedBuffer, GL_STREAM_DRAW);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			//});
		}
	}

	virtual ~NuiGuiVertexBufferMappableImpl()
	{
		relaxToCPU();

		if(_mappedBuffer)
		{
			free(_mappedBuffer);
			_mappedBuffer = NULL;
		}
		_num = 0;
	}

	virtual size_t size() const override
	{
		return _num;
	}

	virtual NuiMappableImpl* clone() const override
	{
		return new NuiGuiVertexBufferMappableImpl(_mappedBuffer, _num, _elementSize);
	}

	virtual const void* map() const override
	{
		return _mappedBuffer;
	}

	virtual void unmap() const override
	{
		
	}

	virtual void* map() override
	{
		if(!_mappedBuffer && (0 != _num))
		{
			_mappedBuffer = malloc(_num*_elementSize);

			//NuiOpenGLThread::instance().enqueueAndWait([&]()
			//{
				glGenBuffers(1, _vbo); // Generate our Vertex Buffer Object
				glBindBuffer(GL_ARRAY_BUFFER, _vbo[0]); // Bind our Vertex Buffer Object
			//});
		}
		return _mappedBuffer;
	}

	virtual void unmap() override
	{
		//NuiOpenGLThread::instance().enqueueAndWait([&]()
		//{
			glBufferData(GL_ARRAY_BUFFER, _num*_elementSize, _mappedBuffer, GL_STREAM_DRAW);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
		//});
	}

	GLuint data()
	{
		return _vbo[0];
	}

	cl_mem clBuffer(const char* name=nullptr)
	{
		//NuiOpenGLThread::instance().enqueueAndWait([&]()
		//{
			if (!NuiGPUMemManager::instance().isValidCLBuffer(_clBuffer))
			{
				_clBuffer = NuiGPUMemManager::instance().CreateCLObjectFromHWBuffer(
					CL_MEM_READ_WRITE, _vbo,
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
			glDeleteBuffers(1, _vbo);
		//});
	}

private:
	GLuint									  _vbo[1];
	size_t									  _num;
	size_t									  _elementSize;
	void*                                     _mappedBuffer;
	cl_mem                                    _clBuffer;
};

// A MIndexBuffer implementation of NuiMappable class
//
class NuiGuiIndexBufferMappableImpl : public NuiMappableImpl
{
public:
	NuiGuiIndexBufferMappableImpl()
		: _mappedBuffer(NULL)
		, _clBuffer(NULL)
		, _num(0)
		, _elementSize(0)
	{}
	NuiGuiIndexBufferMappableImpl(void* buffer, size_t num, size_t elementSize)
		: _clBuffer(NULL)
		, _num(num)
		, _elementSize(elementSize)
	{
		if(_num*_elementSize > 0 && buffer)
		{
			_mappedBuffer = malloc(_num*_elementSize);
			memcpy(_mappedBuffer, buffer, _num*_elementSize);

			//NuiOpenGLThread::instance().enqueueAndWait([&]()
			//{
				glGenBuffers(1, _ibo); // Generate our Index Buffer Object
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ibo[0]); // Bind our Index Buffer Object
				glBufferData(GL_ELEMENT_ARRAY_BUFFER, _num*_elementSize, _mappedBuffer, GL_STREAM_DRAW);
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
			//});
		}
	}

	virtual ~NuiGuiIndexBufferMappableImpl()
	{
		relaxToCPU();

		if(_mappedBuffer)
		{
			free(_mappedBuffer);
			_mappedBuffer = NULL;
		}
		_num = 0;
	}

	virtual size_t size() const override
	{
		return _num;
	}

	virtual NuiMappableImpl* clone() const override
	{
		return new NuiGuiIndexBufferMappableImpl(_mappedBuffer, _num, _elementSize);
	}

	virtual const void* map() const override
	{
		return _mappedBuffer;
	}

	virtual void unmap() const override
	{
	}

	virtual void* map() override
	{
		if(!_mappedBuffer && (0 != _num))
		{
			_mappedBuffer = malloc(_num*_elementSize);

			//NuiOpenGLThread::instance().enqueueAndWait([&]()
			//{
				glGenBuffers(1, _ibo); // Generate our Index Buffer Object
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ibo[0]); // Bind our Index Buffer Object
			//});
		}
		return _mappedBuffer;
	}

	virtual void unmap() override
	{
		//NuiOpenGLThread::instance().enqueueAndWait([&]()
		//{
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, _num*_elementSize, _mappedBuffer, GL_STREAM_DRAW);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		//});
	}

	GLuint data()
	{
		return _ibo[0];
	}

	cl_mem clBuffer(const char* name)
	{
		//NuiOpenGLThread::instance().enqueueAndWait([&]()
		//{
			if (!NuiGPUMemManager::instance().isValidCLBuffer(_clBuffer))
			{
				_clBuffer = NuiGPUMemManager::instance().CreateCLObjectFromHWBuffer(
					CL_MEM_READ_WRITE, _ibo,
					NULL, NuiGPUMemSharedType::XG_GPUMEM_SHARED_IB,
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
			glDeleteBuffers(1, _ibo);
		//});
	}

private:
	GLuint									 _ibo[1];
	size_t									 _num;
	size_t									 _elementSize;
	void*                                    _mappedBuffer;
	cl_mem                                   _clBuffer;
};

// Convert a buffer to hardware buffer
//
struct NuiGuiHWMappable
{
	// Convenient methods to convert to vertex buffer
	static GLuint asPosition3fBuffer (NuiMappable3f& mappable) { return asHWVertexBuffer (mappable); }
	static GLuint asTangent3fBuffer  (NuiMappable3f& mappable) { return asHWVertexBuffer (mappable); }
	static GLuint asNormal3fBuffer	(NuiMappable3f& mappable) { return asHWVertexBuffer (mappable); }
	static GLuint asTexture1fBuffer  (NuiMappablef&  mappable) { return asHWVertexBuffer (mappable); }
	static GLuint asTexture2fBuffer  (NuiMappable2f& mappable) { return asHWVertexBuffer (mappable); }
	static GLuint asTexture3fBuffer  (NuiMappable3f& mappable) { return asHWVertexBuffer (mappable); }
	static GLuint asTexture4fBuffer  (NuiMappable4f& mappable) { return asHWVertexBuffer (mappable); }
	static GLuint asColor4fBuffer    (NuiMappable4f& mappable) { return asHWVertexBuffer (mappable); }

	// Convenient methods to convert to index buffer
	static GLuint asUInt32IndexBuffer(NuiMappableui& mappable) { return asHWIndexBuffer (mappable); }

	// Convenient methods to convert to Viewport 2.0 vertex buffer shared with OpenCL
	static cl_mem asPosition3fBufferCL (NuiMappable3f& mappable) { return asHWVertexBufferSharedWithCL (mappable); }
	static cl_mem asTangent3fBufferCL  (NuiMappable3f& mappable) { return asHWVertexBufferSharedWithCL (mappable); }
	static cl_mem asNormal3fBufferCL   (NuiMappable3f& mappable) { return asHWVertexBufferSharedWithCL (mappable); }
	static cl_mem asTexture1fBufferCL  (NuiMappablef&  mappable) { return asHWVertexBufferSharedWithCL (mappable); }
	static cl_mem asTexture2fBufferCL  (NuiMappable2f& mappable) { return asHWVertexBufferSharedWithCL (mappable); }
	static cl_mem asTexture3fBufferCL  (NuiMappable3f& mappable) { return asHWVertexBufferSharedWithCL (mappable); }
	static cl_mem asTexture4fBufferCL  (NuiMappable4f& mappable) { return asHWVertexBufferSharedWithCL (mappable); }
	static cl_mem asColor4fBufferCL    (NuiMappable4f& mappable) { return asHWVertexBufferSharedWithCL (mappable); }

	// Convenient methods to convert to Viewport 2.0 index buffer shared with OpenCL
	static cl_mem asUInt32IndexBufferCL(NuiMappableui& mappable) { return asHWIndexBufferSharedWithCL  (mappable); }

	template<typename T>
	static GLuint asHWVertexBuffer(NuiMappable<T>& mappable)
	{
		// Get the underlying buffer impl
		std::shared_ptr<NuiGuiVertexBufferMappableImpl> hwImpl =
			NuiMappableAccessor::asImpl<NuiGuiVertexBufferMappableImpl>(mappable);
		if (hwImpl)
			return hwImpl->data();

		// Copy to hardware buffer
		NuiScopedMapConstBuffer<T> buffer(mappable);
		hwImpl.reset(new NuiGuiVertexBufferMappableImpl((void*)(&buffer[0]), buffer.size(), sizeof(T)));

		// Change to hardware buffer in-place
		NuiMappableAccessor::setImpl(mappable, hwImpl);
		return hwImpl->data();
	}

	template<typename T>
	static GLuint asHWIndexBuffer(NuiMappable<T>& mappable)
	{
		// Get the underlying buffer impl
		std::shared_ptr<NuiGuiIndexBufferMappableImpl > hwImpl =
			NuiMappableAccessor::asImpl<NuiGuiIndexBufferMappableImpl >(mappable);
		if (hwImpl)
			return hwImpl->data();

		// Copy to hardware buffer
		NuiScopedMapConstBuffer<T> buffer(mappable);
		hwImpl.reset(new NuiGuiIndexBufferMappableImpl((void*)(&buffer[0]), buffer.size(), sizeof(T)));

		// Make a mappable
		NuiMappableAccessor::setImpl(mappable, hwImpl);
		return hwImpl->data();
	}

	template<typename T>
	static cl_mem asHWVertexBufferSharedWithCL(NuiMappable<T>& mappable)
	{
		if(mappable.size() == 0)
			return NULL;

		// Convert to Viewport 2.0 vertex buffer
		asHWVertexBuffer<T>(mappable);

		// Create OpenCL wrapper object
		std::shared_ptr<NuiGuiVertexBufferMappableImpl> hwImpl =
			NuiMappableAccessor::asImpl<NuiGuiVertexBufferMappableImpl>(mappable);
		return hwImpl ? hwImpl->clBuffer(mappable.name().c_str()) : NULL;
	}

	template<typename T>
	static cl_mem asHWIndexBufferSharedWithCL(NuiMappable<T>& mappable)
	{
		if(mappable.size() == 0)
			return NULL;

		// Convert to index buffer
		asHWIndexBuffer<T>(mappable);

		// Create OpenCL wrapper object
		std::shared_ptr<NuiGuiIndexBufferMappableImpl> hwImpl =
			NuiMappableAccessor::asImpl<NuiGuiIndexBufferMappableImpl>(mappable);
		return hwImpl ? hwImpl->clBuffer(mappable.name().c_str()) : NULL;
	}
};

