#pragma once

#include "OpenCLUtilities/NuiOpenGLThread.h"


// We don't have access to the main thread during EM evaluation. It's not
// friendly to plug-ins that access GPU resources in the compute method.
// VP2/OGS buffer manager expects to execute in the main thread. We are
// left no choice but to use the internal TevaluationManager methods.
//
// Another option is to create a sharing resource context. It works well
// except glDeleteBuffers. glDeleteBuffers will succeed without errors
// but the underlying VBOs are not deleted and cause memory leaks.
//
class NuiMayaOpenGLThreadProvider : public NuiOpenGLThread::Provider
{
public:
    NuiMayaOpenGLThreadProvider();
    virtual ~NuiMayaOpenGLThreadProvider() override;
    virtual void enqueue(NuiOpenGLThread::Task* task) override;
};

