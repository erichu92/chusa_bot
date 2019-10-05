// Generated by gencpp from file chusa_msgs/Sensing3DOnceResponse.msg
// DO NOT EDIT!


#ifndef CHUSA_MSGS_MESSAGE_SENSING3DONCERESPONSE_H
#define CHUSA_MSGS_MESSAGE_SENSING3DONCERESPONSE_H


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
struct Sensing3DOnceResponse_
{
  typedef Sensing3DOnceResponse_<ContainerAllocator> Type;

  Sensing3DOnceResponse_()
    : success(false)  {
    }
  Sensing3DOnceResponse_(const ContainerAllocator& _alloc)
    : success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::chusa_msgs::Sensing3DOnceResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::chusa_msgs::Sensing3DOnceResponse_<ContainerAllocator> const> ConstPtr;

}; // struct Sensing3DOnceResponse_

typedef ::chusa_msgs::Sensing3DOnceResponse_<std::allocator<void> > Sensing3DOnceResponse;

typedef boost::shared_ptr< ::chusa_msgs::Sensing3DOnceResponse > Sensing3DOnceResponsePtr;
typedef boost::shared_ptr< ::chusa_msgs::Sensing3DOnceResponse const> Sensing3DOnceResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::chusa_msgs::Sensing3DOnceResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::chusa_msgs::Sensing3DOnceResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace chusa_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'chusa_msgs': ['/home/jaeho/clear/src/chusa_msgs/msg', '/home/jaeho/clear/src/chusa_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::chusa_msgs::Sensing3DOnceResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::chusa_msgs::Sensing3DOnceResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::chusa_msgs::Sensing3DOnceResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::chusa_msgs::Sensing3DOnceResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::chusa_msgs::Sensing3DOnceResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::chusa_msgs::Sensing3DOnceResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::chusa_msgs::Sensing3DOnceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::chusa_msgs::Sensing3DOnceResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::chusa_msgs::Sensing3DOnceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "chusa_msgs/Sensing3DOnceResponse";
  }

  static const char* value(const ::chusa_msgs::Sensing3DOnceResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::chusa_msgs::Sensing3DOnceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n"
"\n"
;
  }

  static const char* value(const ::chusa_msgs::Sensing3DOnceResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::chusa_msgs::Sensing3DOnceResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Sensing3DOnceResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::chusa_msgs::Sensing3DOnceResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::chusa_msgs::Sensing3DOnceResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CHUSA_MSGS_MESSAGE_SENSING3DONCERESPONSE_H
