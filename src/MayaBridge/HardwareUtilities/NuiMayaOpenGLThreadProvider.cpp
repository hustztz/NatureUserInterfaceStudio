// =======================================================================
// Copyright 2015 Autodesk, Inc. All rights reserved.
//
// This computer source code and related instructions and comments are the
// unpublished confidential  and proprietary information of Autodesk, Inc.
// and are protected under applicable copyright and trade secret law. They 
// may not be disclosed to, copied  or used by any third party without the 
// prior written consent of Autodesk, Inc.
// =======================================================================

#include <DependEngine/TevaluationManager.h>

#include "NuiMayaOpenGLThreadProvider.h"

#include <maya/M3dView.h>

#include <cassert>

namespace
{

// Wrap our tasks as EM's main thread task. 
// 
class MainThreadTask
{
public:
    MainThreadTask(NuiOpenGLThread::Task* task)
        : _task(task)
    {}
    ~MainThreadTask()
    {}

    void operator()()
    {
        // Ensure the shared context is current
        M3dView::active3dView().makeSharedContextCurrent();

        // Execute the task in the main thread
        (*_task)();
        _task->setDone();
    }

    void failed()
    {
        // Ensure the shared context is current
        M3dView::active3dView().makeSharedContextCurrent();

        assert(0 /* MTT failed. Not in EM evaluation? */);
        (*_task)();
        _task->setDone();
    }

private:
    NuiOpenGLThread::Task*   _task;
};

} // anonymous namespace


NuiMayaOpenGLThreadProvider::NuiMayaOpenGLThreadProvider()
{}

NuiMayaOpenGLThreadProvider::~NuiMayaOpenGLThreadProvider()
{}

void NuiMayaOpenGLThreadProvider::enqueue(NuiOpenGLThread::Task* task)
{
    // Enqueue the task for main thread execution
    assert(task);
    TevaluationManager::theOne().enqueueOnMainthread(
        TdgContext::fsNormal, MainThreadTask(task));
}


