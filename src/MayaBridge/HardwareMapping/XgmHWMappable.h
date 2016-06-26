// =======================================================================
// Copyright 2015 Autodesk, Inc. All rights reserved.
//
// This computer source code and related instructions and comments are the
// unpublished confidential  and proprietary information of Autodesk, Inc.
// and are protected under applicable copyright and trade secret law. They 
// may not be disclosed to, copied  or used by any third party without the 
// prior written consent of Autodesk, Inc.
// =======================================================================

#ifndef __XGMHWMAPPABLE_H__
#define __XGMHWMAPPABLE_H__

#include <maya/MHWGeometry.h>
#include <maya/MOpenCLInfo.h>
#include <maya/MViewport2Renderer.h>

#include "OpenCLUtilities/XgMappable.h"
#include "OpenCLUtilities/XgOpenCLUtil.h"
#include "OpenCLUtilities/XgOpenGLThread.h"
#include "OpenCLUtilities/XgGPUMemManager.h"
#include "OpenCLUtilities/NuiDebugMacro.h"

#include <cstring>

// A MVertexBuffer implementation of XgMappable class
//
class XgmVertexBufferMappableImpl : public XgMappableImpl
{
public:
    XgmVertexBufferMappableImpl() {}
    XgmVertexBufferMappableImpl(std::shared_ptr<MHWRender::MVertexBuffer>& data)
        : _data(data), _mappedBuffer(NULL), _clBuffer(NULL)
    {}

    virtual ~XgmVertexBufferMappableImpl()
    {
        XgOpenGLThread::instance().enqueueAndWait([&]()
        {
            relaxToCPU();
            _data = nullptr;
        });
    }

    virtual size_t size() const override
    {
        return _data->vertexCount();
    }

    virtual XgMappableImpl* clone() const override
    {
        XgmVertexBufferMappableImpl* replicant = nullptr;

        XgOpenGLThread::instance().enqueueAndWait([&]()
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
           replicant = new XgmVertexBufferMappableImpl(buffer);
        });

        return replicant;
    }

    virtual const void* map() const override
    {
        const void* pointer = nullptr;

        XgOpenGLThread::instance().enqueueAndWait([&]()
        {
            pointer = const_cast<XgmVertexBufferMappableImpl*>(this)->_data->map();
        });

        return pointer;
    }

    virtual void unmap() const override
    {
        XgOpenGLThread::instance().enqueueAndWait([&]()
        {
            const_cast<XgmVertexBufferMappableImpl*>(this)->_data->unmap();
        });
    }

    virtual void* map() override
    {
        XgOpenGLThread::instance().enqueueAndWait([&]()
        {
            _mappedBuffer = _data->acquire(_data->vertexCount(), false);
        });

        return _mappedBuffer;
    }

    virtual void unmap() override
    {
        XgOpenGLThread::instance().enqueueAndWait([&]()
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
        XgOpenGLThread::instance().enqueueAndWait([&]()
        {
            if (!XgGPUMemManager::instance().isValidCLBuffer(_clBuffer))
            {
                _clBuffer = XgGPUMemManager::instance().CreateCLObjectFromHWBuffer(
                    CL_MEM_READ_WRITE, _data.get()->resourceHandle(),
                    (void*)(_data.get()), XgGPUMemSharedType::XG_GPUMEM_SHARED_VB,
                    name?name:"");
                assert(_clBuffer);
            }
        });
        return _clBuffer;
    }

    void relaxToCPU() override
    {
        XgOpenGLThread::instance().enqueueAndWait([&]()
        {
            if (XgGPUMemManager::instance().isValidCLBuffer(_clBuffer)) {
                // Delete the OpenCL shared object
                cl_int err = XgGPUMemManager::instance().ReleaseMemObjectCL(_clBuffer);
                XGEN_CHECK_CL_ERR(err);
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

// A MIndexBuffer implementation of XgMappable class
//
class XgmIndexBufferMappableImpl : public XgMappableImpl
{
public:
    XgmIndexBufferMappableImpl() {}
    XgmIndexBufferMappableImpl(std::shared_ptr<MHWRender::MIndexBuffer>& data)
        : _data(data), _mappedBuffer(NULL), _clBuffer(NULL)
    {}

    virtual ~XgmIndexBufferMappableImpl()
    {
        XgOpenGLThread::instance().enqueueAndWait([&]()
        {
            relaxToCPU();
            _data = nullptr;
        });
    }

    virtual size_t size() const override
    {
        return _data->size();
    }

    virtual XgMappableImpl* clone() const override
    {
        XgmIndexBufferMappableImpl* replicant = nullptr;

        XgOpenGLThread::instance().enqueueAndWait([&]()
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
            replicant = new XgmIndexBufferMappableImpl(buffer);
        });

        return replicant;
    }

    virtual const void* map() const override
    {
        const void* pointer = nullptr;

        XgOpenGLThread::instance().enqueueAndWait([&]()
        {
            pointer = const_cast<XgmIndexBufferMappableImpl*>(this)->_data->map();
        });

        return pointer;
    }

    virtual void unmap() const override
    {
        XgOpenGLThread::instance().enqueueAndWait([&]()
        {
            const_cast<XgmIndexBufferMappableImpl*>(this)->_data->unmap();
        });
    }

    virtual void* map() override
    {
        XgOpenGLThread::instance().enqueueAndWait([&]()
        {
            _mappedBuffer = _data->acquire(_data->size(), false);
        });
        return _mappedBuffer;
    }

    virtual void unmap() override
    {
        XgOpenGLThread::instance().enqueueAndWait([&]()
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
        XgOpenGLThread::instance().enqueueAndWait([&]()
        {
            if (!XgGPUMemManager::instance().isValidCLBuffer(_clBuffer))
            {
                _clBuffer = XgGPUMemManager::instance().CreateCLObjectFromHWBuffer(
                    CL_MEM_READ_WRITE, _data.get()->resourceHandle(),
                    (void*)(_data.get()), XgGPUMemSharedType::XG_GPUMEM_SHARED_IB,
                    name?name:"");

                assert(_clBuffer);
            }
        });
        return _clBuffer;
    }

    void relaxToCPU() override
    {
        XgOpenGLThread::instance().enqueueAndWait([&]()
        {
            if (XgGPUMemManager::instance().isValidCLBuffer(_clBuffer)) {
                // Delete the OpenCL shared object
                cl_int err = XgGPUMemManager::instance().ReleaseMemObjectCL(_clBuffer);
                XGEN_CHECK_CL_ERR(err);
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
struct XgmHWMappable
{
    // Convenient methods to convert to Viewport 2.0 vertex buffer
    static MHWRender::MVertexBuffer* asPosition3fBuffer (XgMappable3f& mappable) { return asHWVertexBuffer<kPosition3f> (mappable); }
    static MHWRender::MVertexBuffer* asTangent3fBuffer  (XgMappable3f& mappable) { return asHWVertexBuffer<kTangent3f>  (mappable); }
    static MHWRender::MVertexBuffer* asBitangent3fBuffer(XgMappable3f& mappable) { return asHWVertexBuffer<kBitangent3f>(mappable); }
    static MHWRender::MVertexBuffer* asTexture1fBuffer  (XgMappablef&  mappable) { return asHWVertexBuffer<kTexture1f>  (mappable); }
    static MHWRender::MVertexBuffer* asTexture2fBuffer  (XgMappable2f& mappable) { return asHWVertexBuffer<kTexture2f>  (mappable); }
    static MHWRender::MVertexBuffer* asTexture3fBuffer  (XgMappable3f& mappable) { return asHWVertexBuffer<kTexture3f>  (mappable); }
    static MHWRender::MVertexBuffer* asTexture4fBuffer  (XgMappable4f& mappable) { return asHWVertexBuffer<kTexture4f>  (mappable); }
    static MHWRender::MVertexBuffer* asColor4fBuffer    (XgMappable4f& mappable) { return asHWVertexBuffer<kColor4f>    (mappable); }

    // Convenient methods to convert to Viewport 2.0 index buffer
    static MHWRender::MIndexBuffer*  asUInt32IndexBuffer(XgMappableui& mappable) { return asHWIndexBuffer<kUInt32>      (mappable); }

    // Convenient methods to convert to Viewport 2.0 vertex buffer shared with OpenCL
    static cl_mem asPosition3fBufferCL (XgMappable3f& mappable) { return asHWVertexBufferSharedWithCL<kPosition3f> (mappable); }
    static cl_mem asTangent3fBufferCL  (XgMappable3f& mappable) { return asHWVertexBufferSharedWithCL<kTangent3f>  (mappable); }
    static cl_mem asBitangent3fBufferCL(XgMappable3f& mappable) { return asHWVertexBufferSharedWithCL<kBitangent3f>(mappable); }
    static cl_mem asTexture1fBufferCL  (XgMappablef&  mappable) { return asHWVertexBufferSharedWithCL<kTexture1f>  (mappable); }
    static cl_mem asTexture2fBufferCL  (XgMappable2f& mappable) { return asHWVertexBufferSharedWithCL<kTexture2f>  (mappable); }
    static cl_mem asTexture3fBufferCL  (XgMappable3f& mappable) { return asHWVertexBufferSharedWithCL<kTexture3f>  (mappable); }
    static cl_mem asTexture4fBufferCL  (XgMappable4f& mappable) { return asHWVertexBufferSharedWithCL<kTexture4f>  (mappable); }

    // Convenient methods to convert to Viewport 2.0 index buffer shared with OpenCL
    static cl_mem asUInt32IndexBufferCL(XgMappableui& mappable) { return asHWIndexBufferSharedWithCL<kUInt32>      (mappable); }

    // Viewport 2.0 Vertex Buffer Format
    enum VertexFormat
    {
        kPosition3f = 0,
        kTangent3f,
        kBitangent3f,
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
            MVertexBufferDescriptor("", MGeometry::kBitangent, MGeometry::kFloat, 3),
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
    static MHWRender::MVertexBuffer* asHWVertexBuffer(XgMappable<T>& mappable)
    {
        using namespace MHWRender;

        // Get the underlying buffer impl
        std::shared_ptr<XgmVertexBufferMappableImpl> hwImpl =
            XgMappableAccessor::asImpl<XgmVertexBufferMappableImpl>(mappable);
        if (hwImpl)
            return hwImpl->data();

        // Create a hardware vertex buffer
        const MVertexBufferDescriptor& desc = getVertexBufferDesc<VertexFormat>();
        std::shared_ptr<MVertexBuffer> hwBuffer(new MVertexBuffer(desc));

        // Copy to hardware buffer
        XgOpenGLThread::instance().enqueueAndWait([&]()
        {
            XgScopedMapConstBuffer<T> buffer(mappable);

            void* ptr = hwBuffer->acquire(buffer.size(), false);
            memcpy(ptr, &buffer[0], sizeof(T) * buffer.size());
            hwBuffer->commit(ptr);

            hwImpl.reset(new XgmVertexBufferMappableImpl(hwBuffer));
        });

        // Change to hardware buffer in-place
        XgMappableAccessor::setImpl(mappable, hwImpl);
        return hwImpl->data();
    }

    template<int IndexFormat, typename T>
    static MHWRender::MIndexBuffer* asHWIndexBuffer(XgMappable<T>& mappable)
    {
        using namespace MHWRender;

        // Get the underlying buffer impl
        std::shared_ptr<XgmIndexBufferMappableImpl > hwImpl =
            XgMappableAccessor::asImpl<XgmIndexBufferMappableImpl >(mappable);
        if (hwImpl)
            return hwImpl->data();

        // Create a hardware vertex buffer
        const MGeometry::DataType dataType = getIndexBufferType<IndexFormat>();
        std::shared_ptr<MIndexBuffer> hwBuffer(new MIndexBuffer(dataType));

        // Copy to hardware buffer
        XgOpenGLThread::instance().enqueueAndWait([&]()
        {
            XgScopedMapConstBuffer<T> buffer(mappable);

            void* ptr = hwBuffer->acquire(buffer.size(), false);
            memcpy(ptr, &buffer[0], sizeof(T) * buffer.size());
            hwBuffer->commit(ptr);

            hwImpl.reset(new XgmIndexBufferMappableImpl(hwBuffer));
        });
        
        // Make a mappable
        XgMappableAccessor::setImpl(mappable, hwImpl);
        return hwImpl->data();
    }

    template<int VertexFormat, typename T>
    static cl_mem asHWVertexBufferSharedWithCL(XgMappable<T>& mappable)
    {
        // Convert to Viewport 2.0 vertex buffer
        asHWVertexBuffer<VertexFormat,T>(mappable);

        // Create OpenCL wrapper object
        std::shared_ptr<XgmVertexBufferMappableImpl> hwImpl =
            XgMappableAccessor::asImpl<XgmVertexBufferMappableImpl>(mappable);
        return hwImpl ? hwImpl->clBuffer(mappable.name().c_str()) : NULL;
    }

    template<int IndexFormat, typename T>
    static cl_mem asHWIndexBufferSharedWithCL(XgMappable<T>& mappable)
    {
        // Convert to Viewport 2.0 index buffer
        asHWIndexBuffer<IndexFormat,T>(mappable);

        // Create OpenCL wrapper object
        std::shared_ptr<XgmIndexBufferMappableImpl> hwImpl =
            XgMappableAccessor::asImpl<XgmIndexBufferMappableImpl>(mappable);
        return hwImpl ? hwImpl->clBuffer(mappable.name().c_str()) : NULL;
    }
};

#endif // __XGMHWMAPPABLE_H__

