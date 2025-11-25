#pragma once

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>	
#include <sys/socket.h>	
#include <linux/can.h>	
#include <linux/can/raw.h>
#include <pthread.h>
#include <initializer_list>
#include <mutex>
#include <iostream>
#include <cstdlib>
#include <stdint.h>
#include <atomic>

namespace Can {

struct Frame
{
    can_frame canFrame;
    struct can_frame *ptr();
    Frame(unsigned int id=0x12, unsigned char dlc=8);
    Frame &set_id(unsigned int id);
    Frame &set_dlc(unsigned char dlc);
    template<typename ...Args>
    Frame &set_data(Args &&... _args);
    int get_id();
    int get_dlc();
};
template<typename ...Args>
Frame &Frame::set_data(Args &&... _args)
{
    static_assert(((sizeof (Args) + ...) <= 8), "args size is over can_frame maixinum size: 8bytes");
    uint8_t *_dst = canFrame.data;
    int byte_cnt = 0;
    (void)std::initializer_list<int>{([&]{
        if(canFrame.can_dlc < 8)
        {
            byte_cnt+=sizeof(_args);
            if(byte_cnt> canFrame.can_dlc)
            {
                throw std::runtime_error("args size is over can_frame assigned dlc: xbytes");
            }
        }
        memcpy(_dst, std::addressof(_args), sizeof(_args));	
        _dst+=sizeof(_args);
    }(), 0
    )...
    };	
    return *this;
}

class SocketCan
{
public:
    SocketCan();
    SocketCan(std::string canname);
    ~SocketCan();
    void open(std::string canname);
    const std::atomic<bool> &isOpen();

    void send(Frame &frame);
    void send(Frame &&frame);
    void receive(Frame &frame);
    Frame receive();
private:
    std::atomic<bool> is_open;
    Frame rframe;
    int sock;
};

}

// int can_frame_init(can_frame* frame, unsigned int id, unsigned char dlc) {
//     if (frame == nullptr) {
//         std::cerr << "Error: Null pointer passed to can_frame_init." << std::endl;
//         return -1;
//     }
//     if (dlc > CAN_MAX_DLEN) {
//         std::cerr << "Error: Invalid DLC value: " << static_cast<int>(dlc) << ". Maximum DLC is " << CAN_MAX_DLEN << "." << std::endl;
//         return -1;
//     }
//     frame->can_id = id;
//     frame->can_dlc = dlc;

//     memset(frame->data, 0, sizeof(frame->data));
//     return 0;
// }
// int can_send(int& s, can_frame* frame) {
//     if (s < 0) {
//         std::cerr << "Error: Invalid socket descriptor." << std::endl;
//         return -1;
//     }
//     if (frame == nullptr) {
//         std::cerr << "Error: Null pointer passed to can_send." << std::endl;
//         return -1;
//     }
//     ssize_t bytes_sent = write(s, frame, sizeof(struct can_frame));
//     if (bytes_sent < 0) {
//         perror("Error sending CAN frame");
//         return -1;
//     }
//     if (static_cast<size_t>(bytes_sent) != sizeof(struct can_frame)) {
//         std::cerr << "Error: Incomplete CAN frame sent." << std::endl;
//         return -1;
//     }
//     return 0;
// }

// int can_receive(int s, can_frame* frame) {
//     ssize_t bytes_read = read(s, frame, sizeof(struct can_frame));
//     if (bytes_read < 0) {
//         perror("Error receiving CAN frame");
//         return -1;
//     }
//     if (static_cast<size_t>(bytes_read) != sizeof(struct can_frame)) {
//         std::cerr << "Error: Incomplete CAN frame received." << std::endl;
//         return -1;
//     }
//     return 0;
// }

// template<typename ...Args>	
//  void fill_with(void *_dst, Args &&... _args)
// {	
// 	static_assert(((sizeof (Args) + ...) < 64), "args too large");
// 	(void)std::initializer_list<int>{	
// 	([&]{	
// 	  memcpy(_dst, std::addressof(_args),sizeof(_args));	
// 	}( ),0	
// 	)...	
// 	};	
	
// }	



// class CANNode
// {
// public:
//     struct CANData
//     {
//         double x;
//         double y;
//         double z;
//     } can_data;

//     std::mutex data_mutex;  // data锁
//     static CANNode* instance; // 静态实例指针
//     static std::once_flag init_flag; // 确保单例只初始化一次

//     int s_can0;
//     struct sockaddr_can addr_can0;
//     struct ifreq ifr_can0;

//     int s_can1; // 测回环
//     struct sockaddr_can addr_can1;
//     struct ifreq ifr_can1;

//     struct can_frame frame_x;

//     struct can_frame can_frame_receive;
//     CANNode()
//     {
//         if (can_init(s_can0, &addr_can0, &ifr_can0, "can0") < 0)
//         {
//             std::cout << "CAN init error" << std::endl;
//         }
//         else
//         {
//             std::cout << "CAN init success" << std::endl;
//         }

//         can_frame_init(&frame_x, 0x12, 8);

//     }
//     ~CANNode()
//     {
//         if (s_can0 >= 0)
//         {
//             close(s_can0);
//         }
//     }

//     void update_can_data(float x, float y, float z)
//     {
//         std::lock_guard<std::mutex> lock(data_mutex);
//         can_data.x = x;
//         can_data.y = y;
//         can_data.z = z;
//     }

//     static void* send_func(void* arg) {
//         while (true) {
//             int16_t x_val=1, y_val=2, z_val=3;
//             // {   
//             //     // RAII包装类，lock_guard创建时，自动关联互斥锁mutex。作用域是{}.
//             //     std::lock_guard<std::mutex> lock(instance->data_mutex);
//             //     x_val = static_cast<int16_t>(instance->can_data.x);
//             //     y_val = static_cast<int16_t>(instance->can_data.y);
//             //     z_val = static_cast<int16_t>(instance->can_data.z);
//             // }

//             memcpy(instance->frame_x.data, &x_val, sizeof(int16_t));
//             memcpy(instance->frame_x.data+2, &y_val, sizeof(int16_t));
//             memcpy(instance->frame_x.data+4, &z_val, sizeof(int16_t));

//             can_send(instance->s_can0, &instance->frame_x);
//             printf("%f %f %f\n", instance->can_data.x, instance->can_data.y, instance->can_data.z);
//             usleep(100000); // 100ms
//         }
//         return nullptr;
//     }    
//     static void* recv_func(void* arg) {
//         struct can_frame recv_frame;
//         while (true) {
//             if (can_receive(instance->s_can0, &recv_frame) == 0) {
//                 if (recv_frame.can_id == 0xAA) {
//                     std::cout << "Received Data 0xAA, executing activate_can.sh" << std::endl;
//                     // 执行脚本
//                     std::string script_path = "/home/rc/Projects/yolov8/yolov11_tensorrt/scripts/activate_can.sh";
//                     std::string command = "sh " + script_path;
//                     int ret = system(command.c_str());
//                     if (ret == 0) {
//                         std::cout << "activate_can.sh executed successfully." << std::endl;
//                     } else {
//                         std::cerr << "Error executing activate_can.sh (return code: " << ret << ")" << std::endl;
//                     }
//                 } else {
//                     std::cout << "Received CAN ID: 0x" << std::hex << recv_frame.can_id << " Data:";
//                     for (int i = 0; i < recv_frame.can_dlc; ++i) {
//                         std::cout << " " << std::hex << static_cast<int>(recv_frame.data[i]);
//                     }
//                     std::cout << std::dec << std::endl;
//                 }
//             }
//             usleep(50000); // 50ms
//         }
//         return nullptr;
//     }

//     void can_thread_create()
//     {
//         pthread_t send_thread_id;
//         pthread_t recv_thread_id;
//         if (pthread_create(&send_thread_id, nullptr, &CANNode::send_func, nullptr) != 0) {
//             std::cerr << "Send thread creation failed" << std::endl;
//         } else {
//             pthread_detach(send_thread_id); // 线程分离
//         }

//         if (pthread_create(&recv_thread_id, nullptr, &CANNode::recv_func, nullptr) != 0) {
//             std::cerr << "Receive thread creation failed" << std::endl;
//         } else {
//             pthread_detach(recv_thread_id); // 线程分离
//         }
//     }
//     static CANNode* get_instance()
//     {
//         std::call_once(init_flag, [](){ 
//             instance = new CANNode();
//             instance-> can_thread_create();
//          });
//         return instance;
//     }

//     struct CANData get_can_data() 
//     {
//         std::lock_guard<std::mutex> lock(data_mutex);
//         return can_data;
//     }

//     // 禁用拷贝构造和拷贝赋值
//     CANNode(const CANNode&) = delete;
//     CANNode& operator=(const CANNode&) = delete;
// };

// #endif

// #pragma once

// #include <linux/can.h>
// #include <linux/can/raw.h>
// #include <net/if.h>
// #include <sys/ioctl.h>
// #include <sys/socket.h>
// #include <unistd.h>
// #include <cstring>
// #include <iostream>

// inline int can_init(int& socket_fd, struct sockaddr_can* addr, struct ifreq& ifr, const char* can_interface_name) {
//     socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
//     if (socket_fd < 0) {
//         perror("Error while opening socket");
//         return -1;
//     }

//     std::strcpy(ifr.ifr_name, can_interface_name);
//     if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
//         perror("Error getting interface index");
//         return -1;
//     }

//     addr->can_family = AF_CAN;
//     addr->can_ifindex = ifr.ifr_ifindex;

//     if (bind(socket_fd, (struct sockaddr*)addr, sizeof(*addr)) < 0) {
//         perror("Error in socket bind");
//         return -1;
//     }

//     return 0;
// }

// inline int can_send(int socket_fd, struct can_frame* frame) {
//     int nbytes = write(socket_fd, frame, sizeof(struct can_frame));
//     if (nbytes != sizeof(struct can_frame)) {
//         perror("CAN send failed");
//         return -1;
//     }
//     return 0;
// }

// inline int can_receive(int socket_fd, struct can_frame* frame) {
//     int nbytes = read(socket_fd, frame, sizeof(struct can_frame));
//     if (nbytes < 0) {
//         perror("CAN receive failed");
//         return -1;
//     }
//     return 0;
// }
