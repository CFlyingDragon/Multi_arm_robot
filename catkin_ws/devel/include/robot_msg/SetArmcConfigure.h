// Generated by gencpp from file robot_msg/SetArmcConfigure.msg
// DO NOT EDIT!


#ifndef ROBOT_MSG_MESSAGE_SETARMCCONFIGURE_H
#define ROBOT_MSG_MESSAGE_SETARMCCONFIGURE_H

#include <ros/service_traits.h>


#include <robot_msg/SetArmcConfigureRequest.h>
#include <robot_msg/SetArmcConfigureResponse.h>


namespace robot_msg
{

struct SetArmcConfigure
{

typedef SetArmcConfigureRequest Request;
typedef SetArmcConfigureResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetArmcConfigure
} // namespace robot_msg


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::robot_msg::SetArmcConfigure > {
  static const char* value()
  {
    return "e4d1173ab6495a30091da13a92937cf6";
  }

  static const char* value(const ::robot_msg::SetArmcConfigure&) { return value(); }
};

template<>
struct DataType< ::robot_msg::SetArmcConfigure > {
  static const char* value()
  {
    return "robot_msg/SetArmcConfigure";
  }

  static const char* value(const ::robot_msg::SetArmcConfigure&) { return value(); }
};


// service_traits::MD5Sum< ::robot_msg::SetArmcConfigureRequest> should match 
// service_traits::MD5Sum< ::robot_msg::SetArmcConfigure > 
template<>
struct MD5Sum< ::robot_msg::SetArmcConfigureRequest>
{
  static const char* value()
  {
    return MD5Sum< ::robot_msg::SetArmcConfigure >::value();
  }
  static const char* value(const ::robot_msg::SetArmcConfigureRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::robot_msg::SetArmcConfigureRequest> should match 
// service_traits::DataType< ::robot_msg::SetArmcConfigure > 
template<>
struct DataType< ::robot_msg::SetArmcConfigureRequest>
{
  static const char* value()
  {
    return DataType< ::robot_msg::SetArmcConfigure >::value();
  }
  static const char* value(const ::robot_msg::SetArmcConfigureRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::robot_msg::SetArmcConfigureResponse> should match 
// service_traits::MD5Sum< ::robot_msg::SetArmcConfigure > 
template<>
struct MD5Sum< ::robot_msg::SetArmcConfigureResponse>
{
  static const char* value()
  {
    return MD5Sum< ::robot_msg::SetArmcConfigure >::value();
  }
  static const char* value(const ::robot_msg::SetArmcConfigureResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::robot_msg::SetArmcConfigureResponse> should match 
// service_traits::DataType< ::robot_msg::SetArmcConfigure > 
template<>
struct DataType< ::robot_msg::SetArmcConfigureResponse>
{
  static const char* value()
  {
    return DataType< ::robot_msg::SetArmcConfigure >::value();
  }
  static const char* value(const ::robot_msg::SetArmcConfigureResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ROBOT_MSG_MESSAGE_SETARMCCONFIGURE_H
