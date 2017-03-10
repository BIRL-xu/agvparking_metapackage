// Generated by gencpp from file tcp_driver/QRInfo.msg
// DO NOT EDIT!


#ifndef TCP_DRIVER_MESSAGE_QRINFO_H
#define TCP_DRIVER_MESSAGE_QRINFO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace tcp_driver
{
template <class ContainerAllocator>
struct QRInfo_
{
  typedef QRInfo_<ContainerAllocator> Type;

  QRInfo_()
    : x(0)
    , y(0)
    , angle(0.0)
    , TagNum(0)  {
    }
  QRInfo_(const ContainerAllocator& _alloc)
    : x(0)
    , y(0)
    , angle(0.0)
    , TagNum(0)  {
  (void)_alloc;
    }



   typedef int32_t _x_type;
  _x_type x;

   typedef int16_t _y_type;
  _y_type y;

   typedef float _angle_type;
  _angle_type angle;

   typedef int16_t _TagNum_type;
  _TagNum_type TagNum;




  typedef boost::shared_ptr< ::tcp_driver::QRInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tcp_driver::QRInfo_<ContainerAllocator> const> ConstPtr;

}; // struct QRInfo_

typedef ::tcp_driver::QRInfo_<std::allocator<void> > QRInfo;

typedef boost::shared_ptr< ::tcp_driver::QRInfo > QRInfoPtr;
typedef boost::shared_ptr< ::tcp_driver::QRInfo const> QRInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tcp_driver::QRInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tcp_driver::QRInfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace tcp_driver

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'tcp_driver': ['/home/paul/agvParking_ws/src/agvParking_metapackage/tcp_driver/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::tcp_driver::QRInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tcp_driver::QRInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tcp_driver::QRInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tcp_driver::QRInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tcp_driver::QRInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tcp_driver::QRInfo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tcp_driver::QRInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "92b9a26856a02d3c33165b236c56e4c6";
  }

  static const char* value(const ::tcp_driver::QRInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x92b9a26856a02d3cULL;
  static const uint64_t static_value2 = 0x33165b236c56e4c6ULL;
};

template<class ContainerAllocator>
struct DataType< ::tcp_driver::QRInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tcp_driver/QRInfo";
  }

  static const char* value(const ::tcp_driver::QRInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tcp_driver::QRInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 x			#error x\n\
int16 y			#error y\n\
float32 angle	#error angle\n\
int16 TagNum	#QR number.\n\
";
  }

  static const char* value(const ::tcp_driver::QRInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tcp_driver::QRInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.angle);
      stream.next(m.TagNum);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct QRInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tcp_driver::QRInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tcp_driver::QRInfo_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<int32_t>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<int16_t>::stream(s, indent + "  ", v.y);
    s << indent << "angle: ";
    Printer<float>::stream(s, indent + "  ", v.angle);
    s << indent << "TagNum: ";
    Printer<int16_t>::stream(s, indent + "  ", v.TagNum);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TCP_DRIVER_MESSAGE_QRINFO_H
