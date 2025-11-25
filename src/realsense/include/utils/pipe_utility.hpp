#pragma once

#include <functional>
#include <vector>
#include <queue>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <optional>

namespace pipe_utility{

// https://github.com/cameron314/concurrentqueue

enum class ThreadState{
    RUNNING,
    STOPED,
    EXITED
};

// template<typename T>
// using queue_buffer_mt = moodycamel::ConcurrentQueue<T>;

template<typename T>
struct queue_buffer_mt{
    std::queue<T> q;
    std::mutex m;
    size_t buffer_size;
    void resize(size_t size)
    {
        buffer_size = size;
    }
    void enqueue(const T &val)
    {
        std::lock_guard<std::mutex> gurad(m);
        if(q.size() < buffer_size)
            q.push(val);
        else{
            q.pop();
            q.push(val);
        }
    }
    void enqueue(T &&val)
    {
        std::lock_guard<std::mutex> gurad(m);
        if(q.size() < buffer_size)
            q.push(std::move(val));
        else{
            q.pop();
            q.push(std::move(val));
        }
    }
    bool try_dequeue(T &val)
    {
        std::lock_guard<std::mutex> gurad(m);
        if(q.empty())
            return false;
        std::swap(val, q.front());
        q.pop();
        return true;
    }
    std::optional<T> try_dequeue()
    {
        std::lock_guard<std::mutex> gurad(m);
        if(q.empty())
            return {};
        auto val = std::move(q.front());
        q.pop();
        return std::move(val);
    }
    void locked_operation(std::function<void(std::queue<T> &)> callback)
    {
        std::lock_guard<std::mutex> gurad(m);
        callback(q);
    }
};

// sigle input multy output
// https://www.cnblogs.com/fenghualong/p/13855360.html

template<typename Executor, typename Input = typename Executor::Input, typename Output = typename Executor::Output>
class _SIMO;

template<typename Executor>
using SIMO = _SIMO<Executor, typename Executor::Input, typename Executor::Output>;

template<typename ...Processors>
class SignalPub;

struct start_point{};
struct end_point{};

template<typename Executor, typename Output>
class _SIMO<Executor, start_point, Output>{
public:
    using Self=_SIMO<Executor, start_point, Output>;
public:
    using oqueue_type = queue_buffer_mt<Output>;
    void push(Output &product);
    void push(Output &&product);
    _SIMO(size_t buffer_size);
private:
    std::vector<std::unique_ptr<oqueue_type>> _oqueues;
    oqueue_type& _sub_one();
    size_t _buffer_size;

template<typename T1, typename T2, typename T3>
friend class _SIMO;
};

template<typename Executor, typename Output>
_SIMO<Executor, start_point, Output>::_SIMO(size_t buffer_size)
    : _buffer_size(buffer_size){}

template<typename Executor, typename Output>
void _SIMO<Executor, start_point, Output>::push(Output &product)
{
    for(auto & el: _oqueues)
        el->enqueue(product);
}

template<typename Executor, typename Output>
void _SIMO<Executor, start_point, Output>::push(Output &&product)
{
    for(auto & el: _oqueues)
        el->enqueue(std::move(product));
}

template<typename Executor, typename Output>
_SIMO<Executor, start_point, Output>::oqueue_type&
_SIMO<Executor, start_point, Output>::_sub_one()
{
    _oqueues.emplace_back(std::make_unique<oqueue_type>());
    _oqueues.back()->resize(_buffer_size);
    return *(_oqueues.back());
}

template<typename T, typename SFINAE = void>
struct defaultConstructorTrait
{
    static constexpr bool value = false;
};

template<typename T>
struct defaultConstructorTrait<T, std::void_t<decltype(T())>>
{
    static constexpr bool value = true;
};

template<typename Executor, typename Input>
class _SIMO<Executor, Input, end_point>{
public:
    using Self = _SIMO<Executor, Input, end_point>;
// static_assert(defaultConstructorTrait<Input>::value, "Input do not have an default constructor");
static_assert(defaultConstructorTrait<Executor>::value, "class <Executer> does not have an default constructor");
static_assert(
    std::is_same_v<decltype(&Executor::process),
    void(Executor::*)(Input&)>,
    "the signature of function Executor::process is not right"
);
public:
    using iqueue_type = queue_buffer_mt<Input>;
    template<typename Executor_another>
    _SIMO(SIMO<Executor_another> &pre_node, size_t threads_num);
    virtual ~_SIMO();
    void join();
    // running thread's num
    size_t threads_num();
private:
    iqueue_type  &_iqueue;
    // inner wrapper for worker function
    void handle();
    // working threads
    std::vector<std::thread> threads;
    
    std::atomic<ThreadState> threadState;
    std::mutex signalCVMutex;
    std::condition_variable signalCV;
    // running thread num (unexited, unstoped)
    std::atomic<size_t> nthreads;

template<typename T1, typename T2, typename T3>
friend class _SIMO;
template<typename ...Processors>
friend class SignalPub;

};

template<typename Executor, typename Input>
template<typename Executor_another>
_SIMO<Executor, Input, end_point>::_SIMO(SIMO<Executor_another> &pre_node, size_t threads_num)
    : _iqueue(pre_node._sub_one())
    , nthreads(0u)
    , threadState(ThreadState::STOPED)
{
    static_assert(std::is_same_v<typename Executor_another::Output, Input>,
     "Type <Output> of previous node does not fetch Type <Input> of current node");
    for(size_t i = 0u; i < threads_num; i ++)
        threads.emplace_back(&Self::handle, this);
}

template<typename Executor, typename Input>
_SIMO<Executor, Input, end_point>::~_SIMO()
{
    join();
}

template<typename Executor, typename Input>
size_t _SIMO<Executor, Input, end_point>::threads_num()
{
    return nthreads;
}

template<typename Executor, typename Input>
void _SIMO<Executor, Input, end_point>::join()
{
    for(auto &thread: threads)
        if(thread.joinable())
            thread.join();
}

template<typename Executor, typename Input>
void _SIMO<Executor, Input, end_point>::handle()
{
    Executor executor;
    nthreads ++;
    while(threadState != ThreadState::EXITED)
    {
        if(threadState == ThreadState::RUNNING)
        {
            auto item = _iqueue.try_dequeue();
            if(item)
                executor.process(item.value());
        }else if(threadState == ThreadState::STOPED)
        {
            nthreads ++;
            std::unique_lock<std::mutex> lck(signalCVMutex);
            signalCV.wait(lck,
                [this]{return threadState != ThreadState::STOPED;}
            );
            nthreads --;
        }
    }
    nthreads --;
}

template<typename Executor, typename Input, typename Output>
class _SIMO
{
public:
using Self = _SIMO<Executor, Input, Output>;
static_assert(defaultConstructorTrait<Executor>::value, "class <Executer> does not have an default constructor");
static_assert(
    std::is_same_v<decltype(&Executor::process),
    void(Executor::*)(Self &, Input&)>,
    "the signature of function Executor::process is not right"
);
public:
    using iqueue_type = queue_buffer_mt<Input>;
    using oqueue_type = queue_buffer_mt<Output>;
    template<typename Executor_another>
    _SIMO(SIMO<Executor_another> &pre_node, size_t threads_num, size_t buffer_size);

    void push(Output &product);
    void push(Output &&product);
    virtual ~_SIMO();

    void join();
    // running thread's num
    size_t threads_num();
private:
    std::vector<std::unique_ptr<oqueue_type>> _oqueues;
    oqueue_type& _sub_one();
    size_t _buffer_size;

    iqueue_type  &_iqueue;
    // inner wrapper for worker function
    void handle();
    // working threads
    std::vector<std::thread> threads;
    
    std::atomic<ThreadState> threadState;
    std::mutex signalCVMutex;
    std::condition_variable signalCV;
    // running thread num (unexited, unstoped)
    std::atomic<size_t> nthreads;

template<typename T1, typename T2, typename T3>
friend class _SIMO;
template<typename ...Processors>
friend class SignalPub;
};

template<typename Executor, typename Input, typename Output>
template<typename Executor_another>
_SIMO<Executor, Input, Output>::_SIMO(SIMO<Executor_another> &pre_node, size_t threads_num, size_t buffer_size)
    : _buffer_size(buffer_size)
    , _iqueue(pre_node._sub_one())
    , nthreads(0u)
    , threadState(ThreadState::STOPED)
{
    static_assert(std::is_same_v<typename Executor_another::Output, Input>,
     "Type <Output> of previous node does not fetch Type <Input> of current node");
    for(size_t i = 0u; i < threads_num; i ++)
        threads.emplace_back(&Self::handle, this);
}

template<typename Executor, typename Input, typename Output>
void _SIMO<Executor, Input, Output>::push(Output &product)
{
    for(auto & el: _oqueues)
        el->enqueue(product);
}

template<typename Executor, typename Input, typename Output>
void _SIMO<Executor, Input, Output>::push(Output &&product)
{
    for(auto & el: _oqueues)
        el->enqueue(std::move(product));
}

template<typename Executor, typename Input, typename Output>
_SIMO<Executor, Input, Output>::~_SIMO()
{
    join();
}

template<typename Executor, typename Input, typename Output>
_SIMO<Executor, Input, Output>::oqueue_type&
_SIMO<Executor, Input, Output>::_sub_one()
{
    _oqueues.emplace_back(std::make_unique<oqueue_type>());
    _oqueues.back()->resize(_buffer_size);
    return *(_oqueues.back());
}

template<typename Executor, typename Input, typename Output>
size_t _SIMO<Executor, Input, Output>::threads_num()
{
    return nthreads;
}

template<typename Executor, typename Input, typename Output>
void _SIMO<Executor, Input, Output>::join()
{
    for(auto &thread: threads)
        if(thread.joinable())
            thread.join();
}

template<typename Executor, typename Input, typename Output>
void _SIMO<Executor, Input, Output>::handle()
{
    Executor executor;
    nthreads ++;
    while(threadState != ThreadState::EXITED)
    {
        if(threadState == ThreadState::RUNNING)
        {
            auto item = _iqueue.try_dequeue();
            if(item)
                executor.process(*this, item.value());
        }else if(threadState == ThreadState::STOPED)
        {
            nthreads ++;
            std::unique_lock<std::mutex> lck(signalCVMutex);
            signalCV.wait(lck,
                [this]{return threadState != ThreadState::STOPED;}
            );
            nthreads --;
        }
    }
    nthreads --;
}

enum class ThreadSignal{
    START,
    STOP,
    EXIT
};

template<typename ...Processors>
class SignalPub{
public:
    SignalPub(Processors &... processor);
    virtual ~SignalPub();
    void pub(ThreadSignal sig);
    template<typename T>
    void apply_for_each(T callback);
private:
    std::tuple<std::reference_wrapper<Processors>...> processors;
    bool exit_pubed;
};

template<typename ...Processors>
SignalPub<Processors...>::SignalPub(Processors &...processors):
    processors(std::ref(processors)...),
    exit_pubed(false)
{}

template<typename ...Processors>
SignalPub<Processors...>::~SignalPub()
{
    if(exit_pubed == false)
        pub(ThreadSignal::EXIT);
}

template<typename ...Processors>
void SignalPub<Processors...>::pub(ThreadSignal sig)
{
    if(sig == ThreadSignal::START)
        apply_for_each([](auto &element){
            if(element.threadState == ThreadState::EXITED)
                return;
            std::lock_guard<std::mutex> guard(element.signalCVMutex);
            element.threadState = ThreadState::RUNNING;
            element.signalCV.notify_all();
        });
    else if(sig == ThreadSignal::STOP)
        apply_for_each([](auto &element){
            if(element.threadState == ThreadState::EXITED)
                return;
            element.threadState = ThreadState::STOPED;
        });
    else if(sig == ThreadSignal::EXIT){
        exit_pubed = true;
        apply_for_each([](auto &element){
            std::lock_guard<std::mutex> guard(element.signalCVMutex);
            element.threadState = ThreadState::EXITED;
            element.signalCV.notify_all();
        });
    }
}

template<typename ...Processors>
template<typename T>
void SignalPub<Processors...>::apply_for_each(T callback)
{
    std::apply([&callback](auto && ...elementrefs){
        (callback(elementrefs.get()), ...);
    }, processors);
}

// int pipe_example()try
// {
//     auto producer = _SIMO_START<int>(5u);
//     auto consumer1 = _SIMO_END<int, 3u>(producer.oqueues[0],
//         [](auto &self, int products)
//         {
//             std::this_thread::sleep_for(std::chrono::microseconds(100));
//         }
//     );
//     auto sig_pub = SignalPub(consumer1);
//     sig_pub.pub(ThreadSignal::START);
//     std::thread([&producer]{
//         int i = 0;
//         while(true)
//         {
//             producer.push(i);
//             i++;
//             std::this_thread::sleep_for(std::chrono::microseconds(333));
//         }
//     }).detach();
//     char ctrl;
//     while(true)
//     {
//         scanf("%c", &ctrl);
//         if(ctrl == 'p')
//             sig_pub.pub(ThreadSignal::STOP);
//         else if(ctrl == 's')
//             sig_pub.pub(ThreadSignal::START);    
//         else if(ctrl == 'e')
//             sig_pub.pub(ThreadSignal::EXIT);
//         printf("%d\n", consumer1.threads_num());
//     }
// }catch(std::exception &e)
// {
//     std::cout << e.what() << std::endl;
//     return EXIT_FAILURE;
// }

}