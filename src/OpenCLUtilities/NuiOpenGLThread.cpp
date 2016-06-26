// =======================================================================
// Copyright 2015 Autodesk, Inc. All rights reserved.
//
// This computer source code and related instructions and comments are the
// unpublished confidential  and proprietary information of Autodesk, Inc.
// and are protected under applicable copyright and trade secret law. They 
// may not be disclosed to, copied  or used by any third party without the 
// prior written consent of Autodesk, Inc.
// =======================================================================

#include "NuiOpenGLThread.h"

#include <cassert>
#include <thread>


namespace
{

// The main thread is the thread that loads the module
const std::thread::id sMainThreadId = std::this_thread::get_id();

} // anonymous namespace


NuiOpenGLThread& NuiOpenGLThread::instance()
{
    // Singleton
    static NuiOpenGLThread sSingleton;
    return sSingleton;
}

void NuiOpenGLThread::initialize(std::shared_ptr<Provider>&& provider)
{
    // Must be called from the main thread
    assert(std::this_thread::get_id() == sMainThreadId);

    // Obtain the ownership of the provider from the caller
    _provider = std::move(provider);
}

void NuiOpenGLThread::shutdown()
{
    // Destroy the provider
    _provider = nullptr;
}

NuiOpenGLThread::NuiOpenGLThread()
{}

NuiOpenGLThread::~NuiOpenGLThread()
{}

bool NuiOpenGLThread::isMainThread() const
{
    return std::this_thread::get_id() == sMainThreadId;
}

void NuiOpenGLThread::internalEnqueueAndWait(Task* task)
{
    // Enqueue the task for executing in the main thread
    assert(task);
    _provider->enqueue(task);

    // Wait for the task to finish
    while (!task->isDone())
        std::this_thread::yield();
    task->destroy();
}

