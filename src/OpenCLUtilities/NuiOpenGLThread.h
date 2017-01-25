#pragma once

#include <tbb/atomic.h>
#include <tbb/tbb_allocator.h>

#include <memory>


// We don't have access to the main thread during EM evaluation. It's not
// friendly to plug-ins that access GPU resources in the compute method.
// VP2/OGS buffer manager expects to execute in the main thread. We are
// left no choice but to use the internal TevaluationManager methods.
//
// Another option is to create a sharing resource context. It works well
// except glDeleteBuffers. glDeleteBuffers will succeed without errors
// but the underlying VBOs are not deleted and cause memory leaks.
//
class NuiOpenGLThread
{
public:
    // Interface of the main thread task
    class Task
    {
    public:
        // Interface. Override by subclasses.
        Task() { _isDone = false; }
        virtual ~Task() {}

        // Execute the task in the main thread
        virtual void operator()() = 0;

        // Destroy the task body after execution
        virtual void destroy() = 0;

        // Return true if the task finished execution
        bool isDone() const { return _isDone; }

        // Set the task has finished execution
        void setDone() { _isDone = true; }

    private:
        tbb::atomic<bool> _isDone;
    };

    // Template class to help to wrap a lambda as a Task
    template<typename Body>
    class LambdaTask : public Task
    {
    public:
        // Heap is always a bottleneck for multi-threading
        typedef tbb::tbb_allocator<LambdaTask<Body>> AllocatorType;

        // Constructor to wrap a lambda object
        LambdaTask(const Body& body)
            : _body(body)
        {}
        virtual ~LambdaTask() override
        {}

        // Execute the lambda object
        virtual void operator()() override
        {
            _body();
        }

        // Fast destroy by TBB allocator
        virtual void destroy() override
        {
            AllocatorType().destroy(this);
            AllocatorType().deallocate(this, 1);
        }

        // Fast creation by TBB allocator
        static LambdaTask<Body>* construct(const Body& body)
        {
            void* pointer = AllocatorType().allocate(1);
            return new (pointer) LambdaTask<Body>(body);
        }

    private:
        Body _body;
    };

    // Provide the functionality to execute tasks in the main thread.
    class Provider
    {
    public:
        // Interface. Override by subclasses.
        virtual ~Provider() {}

        // Execute the task in the main thread. It's the caller's
        // responsibility to destroy the task.
        virtual void enqueue(NuiOpenGLThread::Task* task) = 0;

		// Make the shared OpenGL context current. Maya/OGS manipulator
		// hijacks OpenGL context and set non-shared context current.
		// We need to restore the shared context in the main thread.
		virtual void makeSharedContextCurrent() = 0;
    };

public:
    // Singleton
    static NuiOpenGLThread& instance();

    // Initialize with the main thread execution provider
    void initialize(std::shared_ptr<Provider>&& provider);

    // Release the main thread execution provider
    void shutdown();

    // Execute the OpenGL commands in the main thread
    template<typename Body>
    void enqueueAndWait(Body body)
    {
        if (isMainThread())
        {
            // Execute immediately when the callling thread is
            // already the main thread
            body();
        }
        else
        {
            // Pending execution in the main thread
            internalEnqueueAndWait(LambdaTask<Body>::construct(body));
        }
    }

private:
    // Singleton. Use XgOpenGLThread::instance() instead.
    NuiOpenGLThread();
    ~NuiOpenGLThread();

    // Return true if the calling thread is the main thread
    bool isMainThread() const;

    // Enqueue a task to execute in the main thread
    void internalEnqueueAndWait(Task* task);

private:
    // Provider of the main thread execution
    std::shared_ptr<Provider>   _provider;
};
