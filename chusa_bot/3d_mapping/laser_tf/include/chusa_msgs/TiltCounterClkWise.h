// Generated by gencpp from file chusa_msgs/TiltCounterClkWise.msg
// DO NOT EDIT!


#ifndef CHUSA_MSGS_MESSAGE_TILTCOUNTERCLKWISE_H
#define CHUSA_MSGS_MESSAGE_TILTCOUNTERCLKWISE_H

#include <ros/service_traits.h>


#include <chusa_msgs/TiltCounterClkWiseRequest.h>
#include <chusa_msgs/TiltCounterClkWiseResponse.h>


namespace chusa_msgs
{

struct TiltCounterClkWise
{

typedef TiltCounterClkWiseRequest Request;
typedef TiltCounterClkWiseResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct TiltCounterClkWise
} // namespace chusa_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::chusa_msgs::TiltCounterClkWise > {
  static const char* value()
  {
    return "676aa7bfb3ec2071e814f2368dfd5fb5";
  }

  static const char* value(const ::chusa_msgs::TiltCounterClkWise&) { return value(); }
};

template<>
struct DataType< ::chusa_msgs::TiltCounterClkWise > {
  static const char* value()
  {
    return "chusa_msgs/TiltCounterClkWise";
  }

  static const char* value(const ::chusa_msgs::TiltCounterClkWise&) { return value(); }
};


// service_traits::MD5Sum< ::chusa_msgs::TiltCounterClkWiseRequest> should match 
// service_traits::MD5Sum< ::chusa_msgs::TiltCounterClkWise > 
template<>
struct MD5Sum< ::chusa_msgs::TiltCounterClkWiseRequest>
{
  static const char* value()
  {
    return MD5Sum< ::chusa_msgs::TiltCounterClkWise >::value();
  }
  static const char* value(const ::chusa_msgs::TiltCounterClkWiseRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::chusa_msgs::TiltCounterClkWiseRequest> should match 
// service_traits::DataType< ::chusa_msgs::TiltCounterClkWise > 
template<>
struct DataType< ::chusa_msgs::TiltCounterClkWiseRequest>
{
  static const char* value()
  {
    return DataType< ::chusa_msgs::TiltCounterClkWise >::value();
  }
  static const char* value(const ::chusa_msgs::TiltCounterClkWiseRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::chusa_msgs::TiltCounterClkWiseResponse> should match 
// service_traits::MD5Sum< ::chusa_msgs::TiltCounterClkWise > 
template<>
struct MD5Sum< ::chusa_msgs::TiltCounterClkWiseResponse>
{
  static const char* value()
  {
    return MD5Sum< ::chusa_msgs::TiltCounterClkWise >::value();
  }
  static const char* value(const ::chusa_msgs::TiltCounterClkWiseResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::chusa_msgs::TiltCounterClkWiseResponse> should match 
// service_traits::DataType< ::chusa_msgs::TiltCounterClkWise > 
template<>
struct DataType< ::chusa_msgs::TiltCounterClkWiseResponse>
{
  static const char* value()
  {
    return DataType< ::chusa_msgs::TiltCounterClkWise >::value();
  }
  static const char* value(const ::chusa_msgs::TiltCounterClkWiseResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // CHUSA_MSGS_MESSAGE_TILTCOUNTERCLKWISE_H
