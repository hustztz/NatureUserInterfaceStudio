#pragma once

#include <Foundation/SgVec4T.h>
#include <Foundation/SgVec3T.h>
#include <Foundation/SgVec2T.h>

#include <cassert>
#include <cstring>
#include <memory>
#include <vector>
#include <limits>

static const size_t MAX_SIZE_T = (std::numeric_limits<size_t>::max)();

// The underlying implementation of a mappable buffer
//
struct NuiMappableImpl
{
    NuiMappableImpl() {}
    virtual ~NuiMappableImpl()
    {
    }

    virtual size_t          size() const = 0;
    virtual NuiMappableImpl* clone() const = 0;

    virtual const void*     map() const = 0;
    virtual void            unmap() const = 0;

    virtual void*           map() = 0;
    virtual void            unmap() = 0;

    virtual void            relaxToCPU() {}
};

// Base class for all mappable classes
class NuiMappableBase
{
public:
    NuiMappableBase(const char* name):
        _name(name), _impl(nullptr)
    {
    }

    NuiMappableBase(const NuiMappableBase& other)
        : _impl(other._impl), _name(other._name)
    {}

    virtual ~NuiMappableBase() {}

    virtual NuiMappableBase* clone() const = 0;

    size_t size() const
    {
        return _impl ? _impl->size() : 0;
    }

    const std::string& name() const { return _name; }

    operator bool() const
    {
        return static_cast<bool>(_impl);
    }

	NuiMappableBase& operator=(const NuiMappableBase& other)
	{
		_name = other._name;
		_impl = other._impl;
		return *this;
	}

protected:
    friend struct NuiMappableAccessor;
    std::shared_ptr<NuiMappableImpl> _impl;
    std::string _name;
};

// A mappable buffer. This class provides access to an array buffer and hides
// storage of the buffer. The buffer data might reside in system memory or
// graphics memory.
//
template<typename T>
class NuiMappable : public NuiMappableBase
{
public:
    NuiMappable(const char* name):
        NuiMappableBase(name)
    {
    }

    NuiMappable(const NuiMappable<T>& other)
        : NuiMappableBase(other)
    {}

    virtual ~NuiMappable()
    {
    }

    virtual NuiMappableBase* clone() const override
    {
        NuiMappable<T>* other = new NuiMappable<T>(_name.c_str());
        other->_impl.reset(_impl->clone());
        return other;
    }

    const T* map() const
    {
        const NuiMappableImpl* constImpl = _impl.get();
        return reinterpret_cast<const T*>(constImpl->map());
    }

    void unmap() const
    {
        const NuiMappableImpl* constImpl = _impl.get();
        constImpl->unmap();
    }

    T* map()
    {
        return reinterpret_cast<T*>(_impl->map());
    }

    void unmap()
    {
        _impl->unmap();
    }

    bool operator!=(const NuiMappable& other)
    {
        return _impl != other._impl;
    }

private:
    friend struct NuiMappableAccessor;
};

// Scoped map/unmap a mappable in RAII fashion
//
template<typename T>
class NuiScopedMapConstBuffer
{
public:
    NuiScopedMapConstBuffer(const NuiMappable<T>& buffer)
        : _buffer(buffer), _pointer(buffer.map()), _size(buffer.size())
    {}

    ~NuiScopedMapConstBuffer()
    {
        _buffer.unmap();
    }

    size_t size() const
    {
        return _size;
    }

    const T& operator[](size_t idx) const
    {
        assert(idx < _buffer.size());
        return _pointer[idx];
    }

    const T* data() const
    {
        assert(_pointer);
        return _pointer;
    }

private:
    const NuiMappable<T>  _buffer;
    const T*             _pointer;
    const size_t         _size;
};

// Scoped map/unmap a mappable in RAII fashion
//
template<typename T>
class NuiScopedMapBuffer
{
public:
    NuiScopedMapBuffer(NuiMappable<T>& buffer)
        : _buffer(buffer), _pointer(buffer.map()), _size(buffer.size())
    {}

    ~NuiScopedMapBuffer()
    {
        _buffer.unmap();
    }

    size_t size() const
    {
        return _size;
    }

    T& operator[](size_t idx)
    {
        assert(idx < _buffer.size());
        return _pointer[idx];
    }

    T* data() const
    {
        assert(_pointer);
        return _pointer;
    }

private:
    NuiMappable<T>   _buffer;
    T*              _pointer;
    const size_t    _size;
};

// A std::vector implementation of XgMappable class
//
template<typename T>
class NuiVectorMappableImpl : public NuiMappableImpl
{
public:
    NuiVectorMappableImpl() {}
    virtual ~NuiVectorMappableImpl() {}

    virtual size_t size() const
    {
        return _data.size();
    }

    virtual NuiMappableImpl* clone() const
    {
        return new NuiVectorMappableImpl<T>(*this);
    }

    virtual const void* map() const
    {
        return &_data[0];
    }

    virtual void unmap() const
    {}

    virtual void* map()
    {
        return &_data[0];
    }

    virtual void unmap()
    {}

    std::vector<T>& data()
    {
        return _data;
    }

private:
    std::vector<T>  _data;
};

// Accessor to the implementation of a mappable buffer
//
struct NuiMappableAccessor
{
    template<typename U, typename T>
    static const std::shared_ptr<U> asImpl(const NuiMappable<T>& mappable)
    {
        return std::dynamic_pointer_cast<U>(mappable._impl);
    }

    template<typename U, typename T>
    static std::shared_ptr<U> asImpl(NuiMappable<T>& mappable)
    {
        return std::dynamic_pointer_cast<U>(mappable._impl);
    }

    template<typename T, typename U>
    static void setImpl(NuiMappable<T>& mappable, std::shared_ptr<U>& impl)
    {
        mappable._impl = std::static_pointer_cast<NuiMappableImpl>(impl);
    }

    template<typename T>
    static void reset(NuiMappable<T>& mappable)
    {
        mappable._impl = nullptr;
    }

    template<typename U, typename T>
    static std::shared_ptr<U> initImpl(NuiMappable<T>& mappable, size_t size = MAX_SIZE_T, const T* initData = nullptr)
    {
        if (!mappable._impl || ((size != MAX_SIZE_T) && (size != mappable._impl->size()))){
            std::shared_ptr<U> ptr(new U());
            setImpl<T, U>(mappable, ptr);
            if (size != MAX_SIZE_T){
                ptr->data().resize(size);
                if (initData) {
                    T* dstData = mappable.map();
                    memcpy((void*)dstData, (void*)initData, sizeof(T)*size);
                    mappable.unmap();
                }
            }
        }
        return asImpl<U, T>(mappable);
    }

    template<typename T>
    static std::shared_ptr<NuiVectorMappableImpl<T> > asVectorImpl(NuiMappable<T>& mappable)
    {
        if (!mappable._impl)
        {
            return initImpl<NuiVectorMappableImpl<T>, T>(mappable);
        }
        else
        {
            std::shared_ptr<NuiVectorMappableImpl<T> > impl = asImpl<NuiVectorMappableImpl<T>, T>(mappable);
            if (!impl)
            {
                impl.reset(new NuiVectorMappableImpl<T>());
                {
                    NuiScopedMapConstBuffer<T> src(mappable);
                    impl->data().resize(src.size());
                    memcpy(&impl->data()[0], &src[0], sizeof(T) * src.size());
                }
                setImpl<T, NuiVectorMappableImpl<T>>(mappable, impl);
            }
            return impl;
        }
    }

    static void relaxToCPU(NuiMappableBase& mappable)
    {
        if(mappable._impl)
        {
            mappable._impl->relaxToCPU();
        }
    }

    template<typename T>
    static void copyData(NuiMappable<T>& mappable, void* dst, unsigned int elemNum)
    {
        NuiScopedMapConstBuffer<T> src(mappable);
        if (elemNum > src.size())
            elemNum = src.size();

        memcpy(dst, &src[0], sizeof(T) * elemNum);
    }
};


// Typedefs
//
typedef NuiMappable<SgVec4f>                     NuiMappable4f;
typedef NuiMappable<SgVec3f>                     NuiMappable3f;
typedef NuiMappable<SgVec2f>                     NuiMappable2f;
typedef NuiMappable<float>                       NuiMappablef;
typedef NuiMappable<unsigned int>                NuiMappableui;
typedef NuiMappable<int>                         NuiMappablei;

typedef NuiScopedMapBuffer<SgVec4f>              NuiScopedMapBuffer4f;
typedef NuiScopedMapBuffer<SgVec3f>              NuiScopedMapBuffer3f;
typedef NuiScopedMapBuffer<SgVec2f>              NuiScopedMapBuffer2f;
typedef NuiScopedMapBuffer<float>                NuiScopedMapBufferf;
typedef NuiScopedMapBuffer<unsigned int>         NuiScopedMapBufferui;
typedef NuiScopedMapBuffer<int>                  NuiScopedMapBufferi;

typedef NuiScopedMapConstBuffer<SgVec4f>         NuiScopedMapConstBuffer4f;
typedef NuiScopedMapConstBuffer<SgVec3f>         NuiScopedMapConstBuffer3f;
typedef NuiScopedMapConstBuffer<SgVec2f>         NuiScopedMapConstBuffer2f;
typedef NuiScopedMapConstBuffer<float>           NuiScopedMapConstBufferf;
typedef NuiScopedMapConstBuffer<unsigned int>    NuiScopedMapConstBufferui;
typedef NuiScopedMapConstBuffer<int>             NuiScopedMapConstBufferi;

typedef NuiVectorMappableImpl<SgVec4f>           NuiVectorMappableImpl4f;
typedef NuiVectorMappableImpl<SgVec3f>           NuiVectorMappableImpl3f;
typedef NuiVectorMappableImpl<SgVec2f>           NuiVectorMappableImpl2f;
typedef NuiVectorMappableImpl<float>             NuiVectorMappableImplf;
typedef NuiVectorMappableImpl<unsigned int>      NuiVectorMappableImplui;
typedef NuiVectorMappableImpl<int>               NuiVectorMappableImpli;

