#include "NuiFrameCache.h"

NuiFrameCache::NuiFrameCache ()
	: NuiFrameCacheImpl()
{
	m_aFramePtrs.SetCapacity(DEVICE_CACHE_SIZE);
}

NuiFrameCache::~NuiFrameCache ()
{
	clear();
}

void NuiFrameCache::clear()
{
	m_aFramePtrs.Clear();
}

bool NuiFrameCache::pushbackFrame(std::shared_ptr<NuiCompositeFrame> pFrame)
{
	if(pFrame)
		return false;

	if( !m_aFramePtrs.PushBack(pFrame) )
	{
		// Memory is full!
		printf_s( "The cache is full!\n" );
		pFrame.reset();
		return false;
	}
	return true;
}

std::shared_ptr<NuiCompositeFrame>	NuiFrameCache::getLatestFrame()
{
	return m_aFramePtrs.GetBackDataPtr();
}

std::shared_ptr<NuiCompositeFrame>	NuiFrameCache::getFrame(UINT frameId)
{
	if(frameId < 0)
		return NULL;

	return m_aFramePtrs.GetDataPtr(frameId);
}