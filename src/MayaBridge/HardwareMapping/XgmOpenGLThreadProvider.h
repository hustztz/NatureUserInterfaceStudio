// =======================================================================
// Copyright 2015 Autodesk, Inc. All rights reserved.
//
// This computer source code and related instructions and comments are the
// unpublished confidential  and proprietary information of Autodesk, Inc.
// and are protected under applicable copyright and trade secret law. They 
// may not be disclosed to, copied  or used by any third party without the 
// prior written consent of Autodesk, Inc.
// =======================================================================

#ifndef __XGMOPENGLTHREADPROVIDER_H__
#define __XGMOPENGLTHREADPROVIDER_H__

#include "OpenCLUtilities/XgOpenGLThread.h"


// We don't have access to the main thread during EM evaluation. It's not
// friendly to plug-ins that access GPU resources in the compute method.
// VP2/OGS buffer manager expects to execute in the main thread. We are
// left no choice but to use the internal TevaluationManager methods.
//
// Another option is to create a sharing resource context. It works well
// except glDeleteBuffers. glDeleteBuffers will succeed without errors
// but the underlying VBOs are not deleted and cause memory leaks.
//
class XgmOpenGLThreadProvider : public XgOpenGLThread::Provider
{
public:
    XgmOpenGLThreadProvider();
    virtual ~XgmOpenGLThreadProvider() override;
    virtual void enqueue(XgOpenGLThread::Task* task) override;
};


#endif // __XGMOPENGLTHREADPROVIDER_H__

