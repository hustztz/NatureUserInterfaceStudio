#pragma once

#include <maya/MHWGeometry.h>
#include <maya/MOpenCLInfo.h>
#include <maya/MViewport2Renderer.h>

#include "OpenCLUtilities/NuiMappable.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"
#include "OpenCLUtilities/NuiOpenGLThread.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"
#include "Foundation/NuiDebugMacro.h"

#include <cstring>

// A MVertexBuffer implementation of NuiMappable class
//
class NuiMayaVertexBufferMappableImpl : public NuiMappableImpl
{
public:
    NuiMayaVertexBufferMappableImpl() {}
    NuiMayaVertexBufferMappableImpl(std::shared_ptr<MHWRender::MVertexBuffer>& data)
        : _data(data), _mappedBuffer(NULL), _clBuffer(NULL)
    {}

    virtual ~NuiMayaVertexBufferMappableImpl()
    {
        NuiOpenGLThread::instance().enqueueAndWait([&]()
        {
            relaxToCPU();
            _data = nullptr;
        });
    }

    virtual size_t size() const override
    {
        return _data->vertexCount();
    }

    virtual NuiMappableImpl* clone() const override
    {
        NuiMayaVertexBufferMappableImpl* replicant = nullptr;

        NuiOpenGLThread::instance().enqueueAndWait([&]()
        {
           const MHWRender::MVertexBufferDescriptor& desc = _data->descriptor();

           const unsigned int size         = _data->vertexCount();
           const unsigned int sizeInBytes  = desc.dataTypeSize() * desc.dimension() * size;

           const void* src = _data->map();

           // Copy data to the new buffer
           std::shared_ptr<MHWRender::MVertexBuffer> buffer(new MHWRender::MVertexBuffer(desc));
           void* dst = buffer->acquire(size, false);
           memcpy(dst, src, sizeInBytes);
           buffer->commit(dst);

           _data->unmap();
           replicant = new NuiMayaVertexBufferMappableImpl(buffer);
        });

        return replicant;
    }

    virtual const void* map() const override
    {
        const void* pointer = nullptr;

        NuiOpenGLThread::instance().enqueueAndWait([&]()
        {
            pointer = const_cast<NuiMayaVertexBufferMappableImpl*>(this)->_data->map();
        });

        return pointer;
    }

    virtual void unmap() const override
    {
        NuiOpenGLThread::instance().enqueueAndWait([&]()
        {
            const_cast<NuiMayaVertexBufferMappableImpl*>(this)->_data->unmap();
        });
    }

    virtual void* map() override
    {
        NuiOpenGLThread::instance().enqueueAndWait([&]()
        {
            _mappedBuffer = _data->acquire(_data->vertexCount(), false);
        });

        return _mappedBuffer;
    }

    virtual void unmap() override
    {
        NuiOpenGLThread::instance().enqueueAndWait([&]()
        {
            _data->commit(_mappedBuffer);
        });
        _mappedBuffer = NULL;
    }

    MHWRender::MVertexBuffer* data()
    {
        return _data.get();
    }

    cl_mem clBuffer(const char* name=nullptr)
    {
        NuiOpenGLThread::instance().enqueueAndWait([&]()
        {
            if (!NuiGPUMemManager::instance().isValidCLBuffer(_clBuffer))
            {
                _clBuffer = NuiGPUMemManager::instance().CreateCLObjectFromHWBuffer(
                    CL_MEM_READ_WRITE, _data.get()->resourceHandle(),
                    (void*)(_data.get()), NuiGPUMemSharedType::XG_GPUMEM_SHARED_VB,
                    name?name:"");
                assert(_clBuffer);
            }
        });
        return _clBuffer;
    }

    void relaxToCPU() override
    {
        NuiOpenGLThread::instance().enqueueAndWait([&]()
        {
            if (NuiGPUMemManager::instance().isValidCLBuffer(_clBuffer)) {
                // Delete the OpenCL shared object
                cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(_clBuffer);
                NUI_CHECK_CL_ERR(err);
                _clBuffer = nullptr;
            }

            // Realize to system memory
            if (_data) {
                _data->unload();
            }
        });
    }

private:
    std::shared_ptr<MHWRender::MVertexBuffer> _data;
    void*                                     _mappedBuffer;
    cl_mem                                    _clBuffer;
};

// A MIndexBuffer implementation of NuiMappable class
//
class NuiMayaIndexBufferMappableImpl : public NuiMappableImpl
{
public:
    NuiMayaIndexBufferMappableImpl() {}
    NuiMayaIndexBufferMappableImpl(std::shared_ptr<MHWRender::MIndexBuffer>& data)
        : _data(data), _mappedBuffer(NULL), _clBuffer(NULL)
    {}

    virtual ~NuiMayaIndexBufferMappableImpl()
    {
        NuiOpenGLThread::instance().enqueueAndWait([&]()
        {
            relaxToCPU();
            _data = nullptr;
        });
    }

    virtual size_t size() const override
    {
        return _data->size();
    }

    virtual NuiMappableImpl* clone() const override
    {
        NuiMayaIndexBufferMappableImpl* replicant = nullptr;

        NuiOpenGLThread::instance().enqueueAndWait([&]()
        {
            assert(_data->dataType() == MHWRender::MGeometry::kUnsignedInt32);
            const unsigned int size         = _data->size();
            const unsigned int sizeInBytes  = sizeof(unsigned int) * size;

            const void* src = _data->map();

            // Copy data to the new buffer
            std::shared_ptr<MHWRender::MIndexBuffer> buffer(new MHWRender::MIndexBuffer(_data->dataType()));
            void* dst = buffer->acquire(size, false);
            memcpy(dst, src, sizeInBytes);
            buffer->commit(dst);

            _data->unmap();
            replicant = new NuiMayaIndexBufferMappableImpl(buffer);
        });

        return replicant;
    }

    virtual const void* map() const override
    {
        const void* pointer = nullptr;

        NuiOpenGLThread::instance().enqueueAndWait([&]()
        {
            pointer = const_cast<NuiMayaIndexBufferMappableImpl*>(this)->_data->map();
        });

        return pointer;
    }

    virtual void unmap() const override
    {
        NuiOpenGLThread::instance().enqueueAndWait([&]()
        {
            const_cast<NuiMayaIndexBufferMappableImpl*>(this)->_data->unmap();
        });
    }

    virtual void* map() override
    {
        NuiOpenGLThread::instance().enqueueAndWait([&]()
        {
            _mappedBuffer = _data->acquire(_data->size(), false);
        });
        return _mappedBuffer;
    }

    virtual void unmap() override
    {
        NuiOpenGLThread::instance().enqueueAndWait([&]()
        {
            _data->commit(_mappedBuffer);
        });
        _mappedBuffer = NULL;
    }

    MHWRender::MIndexBuffer* data()
    {
        return _data.get();
    }

    cl_mem clBuffer(const char* name)
    {
        NuiOpenGLThread::instance().enqueueAndWait([&]()
        {
            if (!NuiGPUMemManager::instance().isValidCLBuffer(_clBuffer))
            {
                _clBuffer = NuiGPUMemManager::instance().CreateCLObjectFromHWBuffer(
                    CL_MEM_READ_WRITE, _data.get()->resourceHandle(),
                    (void*)(_data.get()), NuiGPUMemSharedType::XG_GPUMEM_SHARED_IB,
                    name?name:"");

                assert(_clBuffer);
            }
        });
        return _clBuffer;
    }

    void relaxToCPU() override
    {
        NuiOpenGLThread::instance().enqueueAndWait([&]()
        {
            if (NuiGPUMemManager::instance().isValidCLBuffer(_clBuffer)) {
                // Delete the OpenCL shared object
                cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(_clBuffer);
                NUI_CHECK_CL_ERR(err);
                _clBuffer = nullptr;
            }

            // Realize to system memory
            if (_data) {
                _data->unload();
            }
        });
    }

private:
    std::shared_ptr<MHWRender::MIndexBuffer> _data;
    void*                                    _mappedBuffer;
    cl_mem                                   _clBuffer;
};

// Convert a buffer to hardware buffer
//
struct NuiMayaHWMappable
{
    // Convenient methods to convert to Viewport 2.0 vertex buffer
    static MHWRender::MVertexBuffer* asPosition3fBuffer (NuiMappable3f& mappable) { return asHWVertexBuffer<kPosition3f> (mappable); }
    static MHWRender::MVertexBuffer* asTangent3fBuffer  (NuiMappable3f& mappable) { return asHWVertexBuffer<kTangent3f>  (mappable); }
    static MHWRender::MVertexBuffer* asNormal3fBuffer	(NuiMappable3f& mappable) { return asHWVertexBuffer<kNormal3f>	 (mappable); }
    static MHWRender::MVertexBuffer* asTexture1fBuffer  (NuiMappablef&  mappable) { return asHWVertexBuffer<kTexture1f>  (mappable); }
    static MHWRender::MVertexBuffer* asTexture2fBuffer  (NuiMappable2f& mappable) { return asHWVertexBuffer<kTexture2f>  (mappable); }
    static MHWRender::MVertexBuffer* asTexture3fBuffer  (NuiMappable3f& mappable) { return asHWVertexBuffer<kTexture3f>  (mappable); }
    static MHWRender::MVertexBuffer* asTexture4fBuffer  (NuiMappable4f& mappable) { return asHWVertexBuffer<kTexture4f>  (mappable); }
    static MHWRender::MVertexBuffer* asColor4fBuffer    (NuiMappable4f& mappable) { return asHWVertexBuffer<kColor4f>    (mappable); }

    // Convenient methods to convert to Viewport 2.0 index buffer
    static MHWRender::MIndexBuffer*  asUInt32IndexBuffer(NuiMappableui& mappable) { return asHWIndexBuffer<kUInt32>      (mappable); }

    // Convenient methods to convert to Viewport 2.0 vertex buffer shared with OpenCL
    static cl_mem asPosition3fBufferCL (NuiMappable3f& mappable) { return asHWVertexBufferSharedWithCL<kPosition3f> (mappable); }
    static cl_mem asTangent3fBufferCL  (NuiMappable3f& mappable) { return asHWVertexBufferSharedWithCL<kTangent3f>  (mappable); }
    static cl_mem asNormal3fBufferCL   (NuiMappable3f& mappable) { return asHWVertexBufferSharedWithCL<kNormal3f>	(mappable); }
    static cl_mem asTexture1fBufferCL  (NuiMappablef&  mappable) { return asHWVertexBufferSharedWithCL<kTexture1f>  (mappable); }
    static cl_mem asTexture2fBufferCL  (NuiMappable2f& mappable) { return asHWVertexBufferSharedWithCL<kTexture2f>  (mappable); }
    static cl_mem asTexture3fBufferCL  (NuiMappable3f& mappable) { return asHWVertexBufferSharedWithCL<kTexture3f>  (mappable); }
    static cl_mem asTexture4fBufferCL  (NuiMappable4f& mappable) { return asHWVertexBufferSharedWithCL<kTexture4f>  (mappable); }
	static cl_mem asColor4fBufferCL    (NuiMappable4f& mappable) { return asHWVertexBufferSharedWithCL<kColor4f>  (mappable); }

    // Convenient methods to convert to Viewport 2.0 index buffer shared with OpenCL
    static cl_mem asUInt32IndexBufferCL(NuiMappableui& mappable) { return asHWIndexBufferSharedWithCL<kUInt32>      (mappable); }

    // Viewport 2.0 Vertex Buffer Format
    enum VertexFormat
    {
        kPosition3f = 0,
        kTangent3f,
        kNormal3f,
        kTexture1f,
        kTexture2f,
        kTexture3f,
        kTexture4f,
        kColor4f,
    };

    // Get the vertex format object by the specified index
    template<int VertexFormat>
    static const MHWRender::MVertexBufferDescriptor& getVertexBufferDesc()
    {
        using namespace MHWRender;

        static const MVertexBufferDescriptor sDescriptors[] = {
            MVertexBufferDescriptor("", MGeometry::kPosition,  MGeometry::kFloat, 3),
            MVertexBufferDescriptor("", MGeometry::kTangent,   MGeometry::kFloat, 3),
            MVertexBufferDescriptor("", MGeometry::kNormal,	   MGeometry::kFloat, 3),
            MVertexBufferDescriptor("", MGeometry::kTexture,   MGeometry::kFloat, 1),
            MVertexBufferDescriptor("", MGeometry::kTexture,   MGeometry::kFloat, 2),
            MVertexBufferDescriptor("", MGeometry::kTexture,   MGeometry::kFloat, 3),
            MVertexBufferDescriptor("", MGeometry::kTexture,   MGeometry::kFloat, 4),
            MVertexBufferDescriptor("", MGeometry::kColor,     MGeometry::kFloat, 4),
        };

        return sDescriptors[VertexFormat];
    }

    // Viewport 2.0 Index Buffer Format
    enum IndexBuffer
    {
        kUInt32 = 0,
    };

    // Get the index format object by the specified index
    template<int IndexFormat>
    static const MHWRender::MGeometry::DataType getIndexBufferType()
    {
        using namespace MHWRender;

        static const MGeometry::DataType sIndexTypes[] = {
            MGeometry::kUnsignedInt32,
        };

        return sIndexTypes[IndexFormat];
    }

    template<int VertexFormat, typename T>
    static MHWRender::MVertexBuffer* asHWVertexBuffer(NuiMappable<T>& mappable)
    {
        using namespace MHWRender;

        // Get the underlying buffer impl
        std::shared_ptr<NuiMayaVertexBufferMappableImpl> hwImpl =
            NuiMappableAccessor::asImpl<NuiMayaVertexBufferMappableImpl>(mappable);
        if (hwImpl)
            return hwImpl->data();

        // Create a hardware vertex buffer
        const MVertexBufferDescriptor& desc = getVertexBufferDesc<VertexFormat>();
        std::shared_ptr<MVertexBuffer> hwBuffer(new MVertexBuffer(desc));

        // Copy to hardware buffer
        NuiOpenGLThread::instance().enqueueAndWait([&]()
        {
            NuiScopedMapConstBuffer<T> buffer(mappable);

            void* ptr = hwBuffer->acquire((unsigned int)buffer.size(), false);
            memcpy(ptr, &buffer[0], sizeof(T) * buffer.size());
            hwBuffer->commit(ptr);

            hwImpl.reset(new NuiMayaVertexBufferMappableImpl(hwBuffer));
        });

        // Change to hardware buffer in-place
        NuiMappableAccessor::setImpl(mappable, hwImpl);
        return hwImpl->data();
    }

	template<typename T>
	static MHWRender::MVertexBuffer* asHWVertexBuffer(const MHWRender::MVertexBufferDescriptor& desc, NuiMappable<T>& mappable)
	{
		using namespace MHWRender;

		// Get the underlying buffer impl
		std::shared_ptr<NuiMayaVertexBufferMappableImpl> hwImpl =
			NuiMappableAccessor::asImpl<NuiMayaVertexBufferMappableImpl>(mappable);
		if (hwImpl)
			return hwImpl->data();

		// Create a hardware vertex buffer
		std::shared_ptr<MVertexBuffer> hwBuffer(new MVertexBuffer(desc));

		// Copy to hardware buffer
		NuiOpenGLThread::instance().enqueueAndWait([&]()
		{
			NuiScopedMapConstBuffer<T> buffer(mappable);

			void* ptr = hwBuffer->acquire((unsigned int)buffer.size(), false);
			memcpy(ptr, &buffer[0], sizeof(T) * buffer.size());
			hwBuffer->commit(ptr);

			hwImpl.reset(new NuiMayaVertexBufferMappableImpl(hwBuffer));
		});

		// Change to hardware buffer in-place
		NuiMappableAccessor::setImpl(mappable, hwImpl);
		return hwImpl->data();
	}

    template<int IndexFormat, typename T>
    static MHWRender::MIndexBuffer* asHWIndexBuffer(NuiMappable<T>& mappable)
    {
        using namespace MHWRender;

        // Get the underlying buffer impl
        std::shared_ptr<NuiMayaIndexBufferMappableImpl > hwImpl =
            NuiMappableAccessor::asImpl<NuiMayaIndexBufferMappableImpl >(mappable);
        if (hwImpl)
            return hwImpl->data();

        // Create a hardware vertex buffer
        const MGeometry::DataType dataType = getIndexBufferType<IndexFormat>();
        std::shared_ptr<MIndexBuffer> hwBuffer(new MIndexBuffer(dataType));

        // Copy to hardware buffer
        NuiOpenGLThread::instance().enqueueAndWait([&]()
        {
            NuiScopedMapConstBuffer<T> buffer(mappable);

            void* ptr = hwBuffer->acquire((unsigned int)buffer.size(), false);
            memcpy(ptr, &buffer[0], sizeof(T) * buffer.size());
            hwBuffer->commit(ptr);

            hwImpl.reset(new NuiMayaIndexBufferMappableImpl(hwBuffer));
        });
        
        // Make a mappable
        NuiMappableAccessor::setImpl(mappable, hwImpl);
        return hwImpl->data();
    }

    template<int VertexFormat, typename T>
    static cl_mem asHWVertexBufferSharedWithCL(NuiMappable<T>& mappable)
    {
		if(mappable.size() == 0)
			return NULL;

        // Convert to Viewport 2.0 vertex buffer
        asHWVertexBuffer<VertexFormat,T>(mappable);

        // Create OpenCL wrapper object
        std::shared_ptr<NuiMayaVertexBufferMappableImpl> hwImpl =
            NuiMappableAccessor::asImpl<NuiMayaVertexBufferMappableImpl>(mappable);
        return hwImpl ? hwImpl->clBuffer(mappable.name().c_str()) : NULL;
    }

    template<int IndexFormat, typename T>
    static cl_mem asHWIndexBufferSharedWithCL(NuiMappable<T>& mappable)
    {
		if(mappable.size() == 0)
			return NULL;

        // Convert to Viewport 2.0 index buffer
        asHWIndexBuffer<IndexFormat,T>(mappable);

        // Create OpenCL wrapper object
        std::shared_ptr<NuiMayaIndexBufferMappableImpl> hwImpl =
            NuiMappableAccessor::asImpl<NuiMayaIndexBufferMappableImpl>(mappable);
        return hwImpl ? hwImpl->clBuffer(mappable.name().c_str()) : NULL;
    }
};
