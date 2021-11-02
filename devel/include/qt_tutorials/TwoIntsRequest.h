// Generated by gencpp from file qt_tutorials/TwoIntsRequest.msg
// DO NOT EDIT!


#ifndef QT_TUTORIALS_MESSAGE_TWOINTSREQUEST_H
#define QT_TUTORIALS_MESSAGE_TWOINTSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace qt_tutorials
{
template <class ContainerAllocator>
struct TwoIntsRequest_
{
  typedef TwoIntsRequest_<ContainerAllocator> Type;

  TwoIntsRequest_()
    : a(0)
    , b(0)  {
    }
  TwoIntsRequest_(const ContainerAllocator& _alloc)
    : a(0)
    , b(0)  {
  (void)_alloc;
    }



   typedef int64_t _a_type;
  _a_type a;

   typedef int64_t _b_type;
  _b_type b;





  typedef boost::shared_ptr< ::qt_tutorials::TwoIntsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::qt_tutorials::TwoIntsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct TwoIntsRequest_

typedef ::qt_tutorials::TwoIntsRequest_<std::allocator<void> > TwoIntsRequest;

typedef boost::shared_ptr< ::qt_tutorials::TwoIntsRequest > TwoIntsRequestPtr;
typedef boost::shared_ptr< ::qt_tutorials::TwoIntsRequest const> TwoIntsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::qt_tutorials::TwoIntsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::qt_tutorials::TwoIntsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::qt_tutorials::TwoIntsRequest_<ContainerAllocator1> & lhs, const ::qt_tutorials::TwoIntsRequest_<ContainerAllocator2> & rhs)
{
  return lhs.a == rhs.a &&
    lhs.b == rhs.b;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::qt_tutorials::TwoIntsRequest_<ContainerAllocator1> & lhs, const ::qt_tutorials::TwoIntsRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace qt_tutorials

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::qt_tutorials::TwoIntsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::qt_tutorials::TwoIntsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::qt_tutorials::TwoIntsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::qt_tutorials::TwoIntsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::qt_tutorials::TwoIntsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::qt_tutorials::TwoIntsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::qt_tutorials::TwoIntsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "36d09b846be0b371c5f190354dd3153e";
  }

  static const char* value(const ::qt_tutorials::TwoIntsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x36d09b846be0b371ULL;
  static const uint64_t static_value2 = 0xc5f190354dd3153eULL;
};

template<class ContainerAllocator>
struct DataType< ::qt_tutorials::TwoIntsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "qt_tutorials/TwoIntsRequest";
  }

  static const char* value(const ::qt_tutorials::TwoIntsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::qt_tutorials::TwoIntsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int64 a\n"
"int64 b\n"
;
  }

  static const char* value(const ::qt_tutorials::TwoIntsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::qt_tutorials::TwoIntsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.a);
      stream.next(m.b);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TwoIntsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::qt_tutorials::TwoIntsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::qt_tutorials::TwoIntsRequest_<ContainerAllocator>& v)
  {
    s << indent << "a: ";
    Printer<int64_t>::stream(s, indent + "  ", v.a);
    s << indent << "b: ";
    Printer<int64_t>::stream(s, indent + "  ", v.b);
  }
};

} // namespace message_operations
} // namespace ros

#endif // QT_TUTORIALS_MESSAGE_TWOINTSREQUEST_H
