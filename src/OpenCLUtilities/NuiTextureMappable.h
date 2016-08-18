#pragma once

#include <memory>
#include <cassert>

#include "Shape/NuiImageBuffer.h"

struct NuiTextureMappableImpl
{
	NuiTextureMappableImpl() {}
	virtual ~NuiTextureMappableImpl()
	{
	}

	virtual void		 update(const void* buffer, int width, int height) = 0;
	virtual int          width() const = 0;
	virtual int          height() const = 0;
	virtual void*        data() const = 0;
	virtual NuiTextureMappableImpl* clone() const = 0;

	virtual void            relaxToCPU() {}
};

class NuiTextureMappable
{
public:
	NuiTextureMappable()
		: _impl(nullptr)
	{
	}

	NuiTextureMappable(const NuiTextureMappable& other)
		: _impl(other._impl)
	{}

	~NuiTextureMappable() {}

	NuiTextureMappable* clone() const
	{
		NuiTextureMappable* other = new NuiTextureMappable();
        other->_impl.reset(_impl->clone());
        return other;
	}

	int width() const
	{
		return _impl ? _impl->width() : 0;
	}

	int height() const
	{
		return _impl ? _impl->height() : 0;
	}

	void* data() const
	{
		return _impl ? _impl->data() : NULL;
	}

	operator bool() const
	{
		return static_cast<bool>(_impl);
	}
protected:
	friend struct NuiTextureMappableAccessor;
	std::shared_ptr<NuiTextureMappableImpl> _impl;
};

class NuiImageMappableImpl : public NuiTextureMappableImpl
{
public:
	NuiImageMappableImpl() {}
	virtual ~NuiImageMappableImpl()
	{
		_img.Clear();
	}

	virtual void		 update(const void* buffer, int width, int height) override
	{
		BGRQUAD* pColors = _img.AllocateBuffer(width, height);
		if(pColors && buffer)
			 memcpy(pColors, buffer, sizeof(BGRQUAD)*width*height);
	}
	virtual int          width() const override
	{
		return _img.GetWidth();
	}
	virtual int          height() const override
	{
		return _img.GetHeight();
	}
	virtual void*        data() const override
	{
		return _img.GetBuffer();
	}
	virtual NuiImageMappableImpl* clone() const override
	{
		NuiImageMappableImpl* impl = new NuiImageMappableImpl();
		impl->update(_img.GetBuffer(), _img.GetWidth(), _img.GetHeight());
		return impl;
	}
	
private:
	NuiColorImage  _img;
};

struct NuiTextureMappableAccessor
{
	template<typename U>
	static const std::shared_ptr<U> asImpl(const NuiTextureMappable& mappable)
	{
		return std::dynamic_pointer_cast<U>(mappable._impl);
	}

	template<typename U>
	static std::shared_ptr<U> asImpl(NuiTextureMappable& mappable)
	{
		return std::dynamic_pointer_cast<U>(mappable._impl);
	}

	template<typename U>
	static void setImpl(NuiTextureMappable& mappable, std::shared_ptr<U>& impl)
	{
		mappable._impl = std::static_pointer_cast<NuiTextureMappableImpl>(impl);
	}

	static void reset(NuiTextureMappable& mappable)
	{
		mappable._impl = nullptr;
	}

	static void updateImpl(NuiTextureMappable& mappable, int width, int height, const void* data)
	{
		if(width != mappable.width() || height != mappable.height())
		{
			if(mappable._impl)
				mappable._impl->relaxToCPU();
			mappable._impl = nullptr;
		}

		if(!mappable._impl)
		{
			std::shared_ptr<NuiImageMappableImpl> ptr(new NuiImageMappableImpl());
			setImpl<NuiImageMappableImpl>(mappable, ptr);
		}

		mappable._impl->update(data, width, height);
	}

	static void relaxToCPU(NuiTextureMappable& mappable)
	{
		if(mappable._impl)
		{
			mappable._impl->relaxToCPU();
		}
	}
};