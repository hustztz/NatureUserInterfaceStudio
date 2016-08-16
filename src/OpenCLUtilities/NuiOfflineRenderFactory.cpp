#include "NuiOfflineRenderFactory.h"
#include <assert.h>

FN_initializeOfflineRender NuiOfflineRenderFactory::pInitializeOfflineRenderFn = nullptr;
FN_uninitializeOfflineRender NuiOfflineRenderFactory::pUninitializeOfflineRenderFn = nullptr;
FN_runOfflineRender NuiOfflineRenderFactory::pRunOfflineRenderFn = nullptr;

bool NuiOfflineRenderFactory::initializeOfflineRender(unsigned int vbId)
{
	assert(pInitializeOfflineRenderFn);
	if (!pInitializeOfflineRenderFn) {
		return nullptr;
	}
	return pInitializeOfflineRenderFn(vbId);
}

void NuiOfflineRenderFactory::uninitializeOfflineRender()
{
	assert(pUninitializeOfflineRenderFn);
	if (!pUninitializeOfflineRenderFn) {
		return;
	}
	pUninitializeOfflineRenderFn();
}

void NuiOfflineRenderFactory::runOfflineRender(cl_mem buffer)
{
	assert(pRunOfflineRenderFn);
	if (!pRunOfflineRenderFn) {
		return;
	}
	pRunOfflineRenderFn(buffer);
}