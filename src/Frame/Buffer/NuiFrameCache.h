
#include "NuiDataSharedPtrCache.h"
#include "NuiFrameCacheImpl.h"
#include "Frame/NuiCompositeFrame.h"

class NuiFrameCache : public NuiFrameCacheImpl
{
	static const size_t						DEVICE_CACHE_SIZE = 200;
public:
	NuiFrameCache ();
	virtual ~NuiFrameCache ();

	virtual void								clear() override;

	virtual bool								pushbackFrame(std::shared_ptr<NuiCompositeFrame> pFrame) override;
	virtual std::shared_ptr<NuiCompositeFrame>	getLatestFrame() override;
	virtual std::shared_ptr<NuiCompositeFrame>	getFrame(UINT frameId) override;

private:
	NuiDataSharedPtrCache<NuiCompositeFrame>	m_aFramePtrs;
};