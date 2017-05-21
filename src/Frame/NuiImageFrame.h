#pragma once

#include "NuiGrabberFrame.h"
#include "Shape\NuiImageBuffer.h"
#include "Foundation/NuiFileIOUtilities.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>

template<class imageT, class PixelT>
class NuiImageFrame : public NuiGrabberFrame
{
public:
	NuiImageFrame()
		: NuiGrabberFrame()
		, m_nFPS(0)
	{}
	virtual ~NuiImageFrame() {	Clear(); }

	virtual void	Clear() override	{	m_image.Clear(); }
	void			DeepCopy (const NuiImageFrame& other)
	{
		NuiGrabberFrame::DeepCopy(other);

		m_nFPS = other.m_nFPS;
		m_image = other.m_image;
	}
	NuiImageFrame (const NuiImageFrame& other){ DeepCopy(other); }
	NuiImageFrame& operator = (const NuiImageFrame& other) {	DeepCopy(other); return *this; }

	PixelT*			AllocateBuffer(UINT width, UINT height){	return m_image.AllocateBuffer(width, height);	}
	PixelT*			GetBuffer() const { return m_image.GetBuffer(); }
	UINT			GetWidth() const {	return m_image.GetWidth(); }
	UINT			GetHeight() const { return m_image.GetHeight();	}
	size_t			GetBytesPerPixel() const { return m_image.GetElementBytes(); }
	size_t			GetBufferSize() const { return m_image.GetBufferSize(); }
	size_t			GetBufferNum() const { return m_image.GetBufferNum(); }

	const imageT&	GetImage() const { return m_image; }

	void			SetFPS(double fps) { m_nFPS = fps; }
	double			GetFPS() const { return m_nFPS; }

	void			ReadFrameLock() { m_frameMutex.lock_shared(); }
	void			ReadFrameUnlock() { m_frameMutex.unlock_shared(); }
	void			WriteFrameLock() { m_frameMutex.lock(); }
	void			WriteFrameUnlock() { m_frameMutex.unlock(); }

	bool			loadFrame(const std::string& fileName)
	{
		UINT nWidth = 0;
		UINT nHeight = 0;
		if( NuiFileIOUtilities::readFrameImageHeader(
			fileName,
			&m_liTimeStamp,
			&nWidth,
			&nHeight) )
		{
			UINT nPointNum = nWidth * nHeight;
			if(nPointNum > 0)
			{
				char* pBuffer = reinterpret_cast<char*> (AllocateBuffer(nWidth, nHeight));
				if(pBuffer)
				{
					if( NuiFileIOUtilities::readFrameImageBuffer(
						fileName,
						pBuffer,
						nPointNum * sizeof (PixelT)) )
					{
						return true;
					}
					else
					{
						Clear();
					}
				}
			}
		}
		return false;
	}

	bool			saveFrame(const std::string& fileName, bool bCompressed)
	{
		const UINT nWidth = GetWidth();
		const UINT nHeight = GetHeight();
		const UINT nPointNum = nWidth * nHeight;
		const char* pBuffer = reinterpret_cast<const char*> (GetBuffer());
		if(pBuffer && nPointNum > 0)
		{
			if( NuiFileIOUtilities::writeFrameImage(
				fileName,
				bCompressed,
				m_liTimeStamp,
				nWidth,
				nHeight,
				pBuffer,
				nPointNum * sizeof (PixelT)) )
			{
				return true;
			}
		}
		return false;
	}

private:
	imageT			m_image;
	double			m_nFPS;

	boost::shared_mutex	m_frameMutex;
};

class NuiDepthFrame : public NuiImageFrame<NuiDepthImage, UINT16>
{
public:
	NuiDepthFrame()
		: NuiImageFrame()
		, m_minDepth(500)
		, m_maxDepth(4500)
	{}
	virtual ~NuiDepthFrame(){}

	void		SetMinDepth(UINT16 depth) { m_minDepth = depth; }
	UINT16		GetMinDepth() const { return m_minDepth; }
	void		SetMaxDepth(UINT16 depth) { m_maxDepth = depth; }
	UINT16		GetMaxDepth() const { return m_maxDepth; }

private:
	UINT16			m_minDepth;
	UINT16			m_maxDepth;
};

// Typedefs
//
typedef NuiImageFrame<NuiColorImage, BGRQUAD>                  NuiColorFrame;
typedef NuiImageFrame<NuiDepthDistortionImage, UINT>           NuiDepthDistortionFrame;
typedef NuiImageFrame<NuiBodyIndexImage, BYTE>                 NuiBodyIndexFrame;
typedef NuiImageFrame<NuiColorSpaceImage, ColorSpacePoint>     NuiColorMapFrame;
typedef NuiImageFrame<NuiCameraSpaceImage, CameraSpacePoint>   NuiCameraMapFrame;
typedef NuiImageFrame<NuiDepthSpaceImage, DepthSpacePoint>	   NuiDepthMapFrame;
