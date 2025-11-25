// 这份代码目前没用，被整合到了can.h中。
#ifdef __CANFRAME_H
#define __CANFRAME_H

#include "can.h"
#include <iostream>
#include <pthread.h>
#include <initializer_list>


int s_can0;
struct sockaddr_can addr_can0;
struct ifreq ifr_can0;

int s_can1;
struct sockaddr_can addr_can1;
struct ifreq ifr_can1;

struct can_frame frame_x;
struct can_frame frame_y;
struct can_frame frame_z;

struct can_frame can_frame_receive;

template<typename ...Args>	
 void fill_with(void *_dst, Args &&... _args)
{	
	static_assert(((sizeof (Args) + ...) < 64), "args too large");
	(void)std::initializer_list<int>{	
	([&]{	
	  memcpy(_dst, std::addressof(_args),sizeof(_args));	
	}( ),0	
	)...	
	};	
	
}	

#endif

// #pragma once

// #include <linux/can.h>
// #include <cstring>

// // 初始化 CAN 帧
// inline void can_frame_init(struct can_frame* frame, uint32_t can_id, uint8_t dlc) {
//     frame->can_id = can_id;
//     frame->can_dlc = dlc;
//     std::memset(frame->data, 0, sizeof(frame->data));
// }
