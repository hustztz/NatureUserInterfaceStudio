#include <memory>
#include <cassert>

#include "Shape/NuiImageBuffer.h"

struct NuiTextureMappableImpl
{
	NuiTextureMappableImpl() {}
	virtual ~NuiTextureMappableImpl()
	{
	}

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

	std::shared_ptr<NuiTextureMappableImpl> _impl;
};

class NuiImageMappableBuffer
{
public:
	NuiImageMappableBuffer(const NuiTextureMappable& buffer)
	{
		BGRQUAD* pColors = m_colorImage.AllocateBuffer(buffer.width(), buffer.height());
		if(buffer.data() && pColors)
			memcpy(pColors, buffer.data(), sizeof(BGRQUAD)*buffer.width()*buffer.height());
	}

	~NuiImageMappableBuffer()
	{
		m_colorImage.Clear();
	}

	int width() const
	{
		return m_colorImage.GetWidth();
	}

	int height() const
	{
		return m_colorImage.GetHeight();
	}

	const BGRQUAD* data() const
	{
		assert(buffer.data());
		return buffer.data();
	}

private:
	NuiColorImage		m_colorImage;
};