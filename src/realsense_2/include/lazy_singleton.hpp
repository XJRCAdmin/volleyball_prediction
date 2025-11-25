#pragma once

#include <mutex>
#include <thread>

// https://www.cnblogs.com/xuhuajie/p/11647164.html
// https://www.zhihu.com/question/449263430
// 带有构造函数的单例模式

#define SINGLETON_CTOR(x) \
private:\
    x() = default;\
    x(const x&)=delete;\
    x& operator=(const x&)=delete;\
    ~x()=default;

template<typename T>
class lazy_singleton
{
    SINGLETON_CTOR(lazy_singleton)
public:
    template<typename ...Args>
    static T& instance(Args&& ... args)
    {
        std::call_once(_flag, [&](){
            _ptr = std::make_unique<T>(std::forward<Args...>(args...));
        });
        return *_ptr;
    }
    static T& instance()
    {
        std::call_once(_flag, [&](){
            _ptr = std::make_unique<T>();
        });
        return *_ptr;
    }
private:
    static std::once_flag _flag;
    static std::unique_ptr<T> _ptr;
};

template<typename T>
std::unique_ptr<T> lazy_singleton<T>::_ptr;

template<typename T>
std::once_flag lazy_singleton<T>::_flag;