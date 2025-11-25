#pragma once
#ifndef FLEXSERIAL_IS_INCLUDED
#define FLEXSERIAL_IS_INCLUDED
#include <iostream>
#include <vector>
#include <cstring>
#include <sstream>
#include <cstdio>
#include <map>
#include <atomic>
#include "Serial.h"
#include "boost/asio.hpp"
#include "boost/bind/bind.hpp"
using namespace boost::placeholders;

#if DEBUG_SERIAL_ON == 1
    #define SERIAL_DEBUG
#else
    #define SERIAL_DEBUG if constexpr(false)
#endif

#if SINGLE_DEBUG_ON == 1
namespace FlexSerial{
    class Serial
    {
    public:
        Serial(std::string comport, unsigned int baud=115200U, std::string devideMark = "!"){};
        virtual ~Serial(){};
        template<typename ...Args>
        void send(Args &&...Ax){};
        template<typename ...Args>
        void receive(Args &...Ax){};
        template<typename ...Args>
        bool receiveAsync(Args &...Ax){return 0;}
        template<typename ...Args>
        void receiveNewest(Args &...Ax){};
        template<typename ...Args>
        bool receiveNewestAsync(Args &...Ax){return 0;}
        std::string receive(){return "";}
        std::string receiveAsync(){return "";}
        std::string receiveNewest(){return "";}
        std::string receiveNewestAsync(){return "";}
    };
}
#else

namespace FlexSerial{

using Byte = uint8_t; /*actually unsigned char*/
using Buffer = std::vector<Byte>;
void append_str_buf(Buffer &buf, const std::string &str)
{
    for(const auto &p: str)
		buf.push_back(p);
}
std::string append_buf_str(const Buffer &&buf)
{
    std::string str;
    for(const auto &p: buf)
		str.push_back((char)p);
    return str;
}

/*
* 串口发送的信息将被视作字符串流
* send 与 receiveLatest 函数可以传入任何数量的参数，利用std::cout 与std::cin 相似的规则对字符串流写入与读取值
* 当前串口receiveLatest(a,b,...)的规则是:
* 1. 发送的数据需要以_mark作为结束符号
*     eg.   假如你需要一次接受的数据为 "123" "456" "789"，设置_mark="!"，则
*           调用函数的方式：receiveLatest(<std::string>str1, str2, str3)
*           接受原始数据的初始内容："123 456 789!"
*    于是对于传输流数据 在每组数据之后发送一个_mark
*    这样就可以循环中接受同一格式的数据组流
* 2. 在接受到标志结束的_mark之前，串口将一直保持阻塞状态，等待数据到达并返回
* 3. 对于一个循环周期收到多组数据的情况，将会返回最新（最靠后）的一组
*/
class Serial
{
    static const u_int  max_bytes_num = 1000000, 
                        bytes_num = 2,  /*for receive*/
                        timeout = 1;    /*ms*/

private:
    std::string         _comport;
    unsigned int        _baud;
    std::string         _mark;
    serial::Serial      _s;
	Buffer              _obuf;
    std::string         _ibuf;
    std::stringstream   oss, iss; /*temporary*/
    
    void send();
public:
    Serial(std::string comport, unsigned int baud=115200U, std::string devideMark = "!");
    virtual ~Serial();
    template<typename ...Args>
    void send(Args &&...Ax);
    template<typename ...Args>
    void receive(Args &...Ax);
    template<typename ...Args>
    bool receiveAsync(Args &...Ax);
    template<typename ...Args>
    void receiveNewest(Args &...Ax);
    template<typename ...Args>
    bool receiveNewestAsync(Args &...Ax);
    std::string receive();
    std::string receiveAsync();
    std::string receiveNewest();
    std::string receiveNewestAsync();
};

Serial::Serial(std::string comport, unsigned int baud, std::string devideMark)
{
    _comport = comport;
    _baud = baud;
    _mark = devideMark;
    iss.clear();
    iss.str(std::string());
    _obuf.clear();
    _ibuf="";
    _s.open(_comport, _baud);
}

Serial::~Serial()
{
    _s.close();
}

template<typename ...Args>
void Serial::send(Args &&...Ax)
{
    oss.clear();
    oss.str(std::string());
    (void)std::initializer_list<int>{(oss << Ax, 0)... };
    Serial::send();
}

void Serial::send()
{
    SERIAL_DEBUG std::cout<<"<send>"<<oss.str()<<std::endl;
    _obuf.clear();
    append_str_buf(_obuf, oss.str());
    _s.transmit(_obuf);
}

std::string Serial::receive()
{
    iss.clear();
    iss.str(_ibuf);
    int end_pos;
    std::string ts;
    do{
        iss << ts;
        ts.assign(append_buf_str(_s.receiveAsync(bytes_num, timeout).get()));  /*important time factor*/
        end_pos = ts.find_last_of(_mark);
    }while(end_pos == -1);
    iss << ts.substr(0, end_pos);
    _ibuf.assign(ts.substr(end_pos+1, ts.length()-end_pos-1));
    SERIAL_DEBUG std::cout<<"<recv>"<<iss.str()<<std::endl;
    return iss.str();
}

std::string Serial::receiveAsync()
{
    iss.clear();
    iss.str(_ibuf);
    int end_pos;
    std::string ts;
    do{
        iss << ts;
        ts.assign(append_buf_str(_s.receiveAsync(bytes_num, timeout).get()));  /*important time factor*/
        end_pos = ts.find_last_of(_mark);
    }while(ts.length() != 0 && end_pos == -1);
    if(ts.length() == 0){
        _ibuf.assign(iss.str());
        // SERIAL_DEBUG std::cout<<"<recv>"<<iss.str()<<std::endl;
        return "";
    }
    iss << ts.substr(0, end_pos);
    _ibuf.assign(ts.substr(end_pos+1, ts.length()-end_pos-1));
    SERIAL_DEBUG std::cout<<"<recv>"<<iss.str()<<std::endl;
    return iss.str();
}

std::string Serial::receiveNewest()
{
    std::string ts;
    do{
        ts.assign(append_buf_str(_s.receiveAsync(max_bytes_num, timeout).get()));
    }while(ts.length() == max_bytes_num);
    int st_pos = ts.find_last_of(_mark);
    if(st_pos != -1)
        _ibuf.assign(ts.substr(st_pos+1, ts.length()-st_pos-1));
    return Serial::receive();
}

std::string Serial::receiveNewestAsync()
{
    std::string ts;
    do{
        ts.assign(append_buf_str(_s.receiveAsync(max_bytes_num, timeout).get()));
    }while(ts.length() == max_bytes_num);
    if(ts.length() == 0)
        return "";
    int st_pos = ts.find_last_of(_mark);
    if(st_pos != -1)
        _ibuf.assign(ts.substr(st_pos+1, ts.length()-st_pos-1));
    else
        _ibuf.assign(ts);
    return Serial::receiveAsync();
}

template<typename ...Args>
void Serial::receive(Args &...Ax)
{
    Serial::receive();
    (void)std::initializer_list<int>{(iss >> Ax, 0)... };
}

template<typename ...Args>
bool Serial::receiveAsync(Args &...Ax)
{
    if(Serial::receiveAsync().length() == 0)
        return false;
    (void)std::initializer_list<int>{(iss >> Ax, 0)... };
    return true;
}

template<typename ...Args>
void Serial::receiveNewest(Args &...Ax)
{
    Serial::receiveNewest();
    (void)std::initializer_list<int>{(iss >> Ax, 0)... };
}

template<typename ...Args>
bool Serial::receiveNewestAsync(Args &...Ax)
{
    if(Serial::receiveNewestAsync().length() == 0)
        return false;
    (void)std::initializer_list<int>{(iss >> Ax, 0)... };
    return true;
}

}

#endif

#endif
