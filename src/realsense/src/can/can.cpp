#include "can.hpp"

namespace Can {

struct can_frame *Frame::ptr()
{
    return &canFrame;
}
Frame::Frame(unsigned int id, unsigned char dlc)
{
    memset(canFrame.data, 0, 8);
    set_id(id).set_dlc(dlc);
}
Frame &Frame::set_id(unsigned int id)
{
    if(id >= 0x800)
    {
        throw std::runtime_error("invalid can id value: x");
    }
    canFrame.can_id = id;
    return *this;
}
Frame &Frame::set_dlc(unsigned char dlc)
{
    if(dlc > 8)
    {
        throw std::runtime_error("invalid dlc value: x");
    }
    canFrame.can_dlc = dlc;
    return *this;
}
int Frame::get_id()
{
    return canFrame.can_id;
}
int Frame::get_dlc()
{
    return canFrame.can_dlc;
}

SocketCan::SocketCan(){}
SocketCan::SocketCan(std::string canname)
{
    open(canname);
}
void SocketCan::open(std::string canname)
{
    if (!canname.size() || canname.size() >= IFNAMSIZ) {
        throw std::runtime_error("Error: Invalid CAN interface name.");
    }
    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0)
    {
        throw std::runtime_error("Error: creating Can Socket failed.");
    }
    is_open = true;
    sockaddr_can addr;
    ifreq ifr;
    strcpy(ifr.ifr_name, canname.c_str());
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        is_open = false;
        close(sock);
        throw std::runtime_error("Error getting CAN interface index");
    }
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        is_open = false;
        close(sock);
        throw std::runtime_error("Error binding socket");
    }
}
SocketCan::~SocketCan()
{
    if (is_open)
    {
        is_open = false;
        close(sock);
    }
}
const std::atomic<bool> &SocketCan::isOpen()
{
    return is_open;
}
void SocketCan::send(Frame &frame)
{
    if (!is_open)
    {
        throw std::runtime_error("Error: SocketCan is not open.");
    }
    ssize_t bytes_sent = write(sock, frame.ptr(), sizeof(struct can_frame));
    if (bytes_sent < 0)
    {
        throw std::runtime_error("Error: SocketCan send failed.");
    }
    if (bytes_sent != sizeof(struct can_frame))
    {
        throw std::runtime_error("Error: Incomplete CAN frame sent.");
    }
}
void SocketCan::receive(Frame &frame)
{ 
    ssize_t bytes_received = read(sock, &frame, sizeof(struct can_frame));
    if (bytes_received < 0)
    {
        throw std::runtime_error("Error: SocketCan receive failed.");
    }
    if (bytes_received != sizeof(struct can_frame))
    {
        throw std::runtime_error("Error: Incomplete CAN frame received.");
    }
}
void SocketCan::send(Frame &&frame)
{
    this->send(frame);
}
Frame SocketCan::receive()
{
    this->receive(rframe);
    return rframe;
}

}