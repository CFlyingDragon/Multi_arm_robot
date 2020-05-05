#ifndef SERIAL_OPEN_H
#define SERIAL_OPEN_H

#include <sstream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
using namespace boost::asio; 

void serial_open(std::string serial_device,long bauld) 
{ 
  io_service iosev;
        //节点文件
  serial_port sp(iosev, serial_device);
        // 设置参数
  sp.set_option(serial_port::baud_rate(bauld));
  sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
  sp.set_option(serial_port::parity(serial_port::parity::none));
  sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
  sp.set_option(serial_port::character_size(8));
}
#endif
