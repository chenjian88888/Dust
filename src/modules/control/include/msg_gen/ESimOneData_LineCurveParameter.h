// Generated by gencpp from file msg_gen/ESimOneData_LineCurveParameter.msg
// DO NOT EDIT!


#ifndef MSG_GEN_MESSAGE_ESIMONEDATA_LINECURVEPARAMETER_H
#define MSG_GEN_MESSAGE_ESIMONEDATA_LINECURVEPARAMETER_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <msg_gen/SimOneData_Vec3f.h>
#include <msg_gen/SimOneData_Vec3f.h>

namespace msg_gen
{
template <class ContainerAllocator>
struct ESimOneData_LineCurveParameter_
{
  typedef ESimOneData_LineCurveParameter_<ContainerAllocator> Type;

  ESimOneData_LineCurveParameter_()
    : C0(0.0)
    , C1(0.0)
    , C2(0.0)
    , C3(0.0)
    , firstPoints()
    , endPoints()
    , length(0.0)  {
    }
  ESimOneData_LineCurveParameter_(const ContainerAllocator& _alloc)
    : C0(0.0)
    , C1(0.0)
    , C2(0.0)
    , C3(0.0)
    , firstPoints(_alloc)
    , endPoints(_alloc)
    , length(0.0)  {
  (void)_alloc;
    }



   typedef double _C0_type;
  _C0_type C0;

   typedef double _C1_type;
  _C1_type C1;

   typedef double _C2_type;
  _C2_type C2;

   typedef double _C3_type;
  _C3_type C3;

   typedef  ::msg_gen::SimOneData_Vec3f_<ContainerAllocator>  _firstPoints_type;
  _firstPoints_type firstPoints;

   typedef  ::msg_gen::SimOneData_Vec3f_<ContainerAllocator>  _endPoints_type;
  _endPoints_type endPoints;

   typedef double _length_type;
  _length_type length;





  typedef boost::shared_ptr< ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator> const> ConstPtr;

}; // struct ESimOneData_LineCurveParameter_

typedef ::msg_gen::ESimOneData_LineCurveParameter_<std::allocator<void> > ESimOneData_LineCurveParameter;

typedef boost::shared_ptr< ::msg_gen::ESimOneData_LineCurveParameter > ESimOneData_LineCurveParameterPtr;
typedef boost::shared_ptr< ::msg_gen::ESimOneData_LineCurveParameter const> ESimOneData_LineCurveParameterConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator1> & lhs, const ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator2> & rhs)
{
  return lhs.C0 == rhs.C0 &&
    lhs.C1 == rhs.C1 &&
    lhs.C2 == rhs.C2 &&
    lhs.C3 == rhs.C3 &&
    lhs.firstPoints == rhs.firstPoints &&
    lhs.endPoints == rhs.endPoints &&
    lhs.length == rhs.length;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator1> & lhs, const ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace msg_gen

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator> >
{
  static const char* value()
  {
    return "062cae6112a15a407e88906c24a66a71";
  }

  static const char* value(const ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x062cae6112a15a40ULL;
  static const uint64_t static_value2 = 0x7e88906c24a66a71ULL;
};

template<class ContainerAllocator>
struct DataType< ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator> >
{
  static const char* value()
  {
    return "msg_gen/ESimOneData_LineCurveParameter";
  }

  static const char* value(const ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 C0\n"
"float64 C1\n"
"float64 C2\n"
"float64 C3\n"
"SimOneData_Vec3f firstPoints\n"
"SimOneData_Vec3f endPoints\n"
"float64 length\n"
"================================================================================\n"
"MSG: msg_gen/SimOneData_Vec3f\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
;
  }

  static const char* value(const ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.C0);
      stream.next(m.C1);
      stream.next(m.C2);
      stream.next(m.C3);
      stream.next(m.firstPoints);
      stream.next(m.endPoints);
      stream.next(m.length);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ESimOneData_LineCurveParameter_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::msg_gen::ESimOneData_LineCurveParameter_<ContainerAllocator>& v)
  {
    s << indent << "C0: ";
    Printer<double>::stream(s, indent + "  ", v.C0);
    s << indent << "C1: ";
    Printer<double>::stream(s, indent + "  ", v.C1);
    s << indent << "C2: ";
    Printer<double>::stream(s, indent + "  ", v.C2);
    s << indent << "C3: ";
    Printer<double>::stream(s, indent + "  ", v.C3);
    s << indent << "firstPoints: ";
    s << std::endl;
    Printer< ::msg_gen::SimOneData_Vec3f_<ContainerAllocator> >::stream(s, indent + "  ", v.firstPoints);
    s << indent << "endPoints: ";
    s << std::endl;
    Printer< ::msg_gen::SimOneData_Vec3f_<ContainerAllocator> >::stream(s, indent + "  ", v.endPoints);
    s << indent << "length: ";
    Printer<double>::stream(s, indent + "  ", v.length);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MSG_GEN_MESSAGE_ESIMONEDATA_LINECURVEPARAMETER_H
