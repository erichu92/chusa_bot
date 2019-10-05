// Generated by gencpp from file chusa_msgs/3DSensingOnceResponse.msg
// DO NOT EDIT!


#ifndef CHUSA_MSGS_MESSAGE_3DSENSINGONCERESPONSE_H
#define CHUSA_MSGS_MESSAGE_3DSENSINGONCERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace chusa_msgs
{
template <class ContainerAllocator>
struct 3DSensingOnceResponse_
{
  typedef 3DSensingOnceResponse_<ContainerAllocator> Type;

  3DSensingOnceResponse_()
    : return(false)  {
    }
  3DSensingOnceResponse_(const ContainerAllocator& _alloc)
    : return(false)  {
  (void)_alloc;
    }



   typedef uint8_t _return_type;
  _return_type return;





  typedef boost::shared_ptr< ::chusa_msgs::3DSensingOnceResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::chusa_msgs::3DSensingOnceResponse_<ContainerAllocator> const> ConstPtr;

}; // struct 3DSensingOnceResponse_

typedef ::chusa_msgs::3DSensingOnceResponse_<std::allocator<void> > 3DSensingOnceResponse;

typedef boost::shared_ptr< ::chusa_msgs::3DSensingOnceResponse > 3DSensingOnceResponsePtr;
typedef boost::shared_ptr< ::chusa_msgs::3DSensingOnceResponse const> 3DSensingOnceResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::chusa_msgs::3DSensingOnceResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::chusa_msgs::3DSensingOnceResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace chusa_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'chusa_msgs': ['/home/jaeho/pi/src/chusa_msgs/msg', '/home/jaeho/pi/src/chusa_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::chusa_msgs::3DSensingOnceResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::chusa_msgs::3DSensingOnceResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::chusa_msgs::3DSensingOnceResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::chusa_msgs::3DSensingOnceResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::chusa_msgs::3DSensingOnceResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::chusa_msgs::3DSensingOnceResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::chusa_msgs::3DSensingOnceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "88eeb9c2a71b8e1ccaf759811ce45dd0";
  }

  static const char* value(const ::chusa_msgs::3DSensingOnceResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x88eeb9c2a71b8e1cULL;
  static const uint64_t static_value2 = 0xcaf759811ce45dd0ULL;
};

template<class ContainerAllocator>
struct DataType< ::chusa_msgs::3DSensingOnceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "chusa_msgs/3DSensingOnceResponse";
  }

  static const char* value(const ::chusa_msgs::3DSensingOnceResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::chusa_msgs::3DSensingOnceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool return\n"
"\n"
;
  }

  static const char* value(const ::chusa_msgs::3DSensingOnceResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::chusa_msgs::3DSensingOnceResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.return);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct 3DSensingOnceResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::chusa_msgs::3DSensingOnceResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::chusa_msgs::3DSensingOnceResponse_<ContainerAllocator>& v)
  {
    s << indent << "return: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.return);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CHUSA_MSGS_MESSAGE_3DSENSINGONCERESPONSE_H