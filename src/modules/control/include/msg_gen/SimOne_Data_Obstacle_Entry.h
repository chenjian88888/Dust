// Generated by gencpp from file msg_gen/SimOne_Data_Obstacle_Entry.msg
// DO NOT EDIT!


#ifndef MSG_GEN_MESSAGE_SIMONE_DATA_OBSTACLE_ENTRY_H
#define MSG_GEN_MESSAGE_SIMONE_DATA_OBSTACLE_ENTRY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <msg_gen/SimOne_Obstacle_Type.h>

namespace msg_gen
{
template <class ContainerAllocator>
struct SimOne_Data_Obstacle_Entry_
{
  typedef SimOne_Data_Obstacle_Entry_<ContainerAllocator> Type;

  SimOne_Data_Obstacle_Entry_()
    : id(0)
    , viewId(0)
    , type()
    , theta(0.0)
    , posX(0.0)
    , posY(0.0)
    , posZ(0.0)
    , oriX(0.0)
    , oriY(0.0)
    , oriZ(0.0)
    , velX(0.0)
    , velY(0.0)
    , velZ(0.0)
    , length(0.0)
    , width(0.0)
    , height(0.0)  {
    }
  SimOne_Data_Obstacle_Entry_(const ContainerAllocator& _alloc)
    : id(0)
    , viewId(0)
    , type(_alloc)
    , theta(0.0)
    , posX(0.0)
    , posY(0.0)
    , posZ(0.0)
    , oriX(0.0)
    , oriY(0.0)
    , oriZ(0.0)
    , velX(0.0)
    , velY(0.0)
    , velZ(0.0)
    , length(0.0)
    , width(0.0)
    , height(0.0)  {
  (void)_alloc;
    }



   typedef int32_t _id_type;
  _id_type id;

   typedef int32_t _viewId_type;
  _viewId_type viewId;

   typedef  ::msg_gen::SimOne_Obstacle_Type_<ContainerAllocator>  _type_type;
  _type_type type;

   typedef double _theta_type;
  _theta_type theta;

   typedef double _posX_type;
  _posX_type posX;

   typedef double _posY_type;
  _posY_type posY;

   typedef double _posZ_type;
  _posZ_type posZ;

   typedef double _oriX_type;
  _oriX_type oriX;

   typedef double _oriY_type;
  _oriY_type oriY;

   typedef double _oriZ_type;
  _oriZ_type oriZ;

   typedef double _velX_type;
  _velX_type velX;

   typedef double _velY_type;
  _velY_type velY;

   typedef double _velZ_type;
  _velZ_type velZ;

   typedef double _length_type;
  _length_type length;

   typedef double _width_type;
  _width_type width;

   typedef double _height_type;
  _height_type height;





  typedef boost::shared_ptr< ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator> const> ConstPtr;

}; // struct SimOne_Data_Obstacle_Entry_

typedef ::msg_gen::SimOne_Data_Obstacle_Entry_<std::allocator<void> > SimOne_Data_Obstacle_Entry;

typedef boost::shared_ptr< ::msg_gen::SimOne_Data_Obstacle_Entry > SimOne_Data_Obstacle_EntryPtr;
typedef boost::shared_ptr< ::msg_gen::SimOne_Data_Obstacle_Entry const> SimOne_Data_Obstacle_EntryConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator1> & lhs, const ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id &&
    lhs.viewId == rhs.viewId &&
    lhs.type == rhs.type &&
    lhs.theta == rhs.theta &&
    lhs.posX == rhs.posX &&
    lhs.posY == rhs.posY &&
    lhs.posZ == rhs.posZ &&
    lhs.oriX == rhs.oriX &&
    lhs.oriY == rhs.oriY &&
    lhs.oriZ == rhs.oriZ &&
    lhs.velX == rhs.velX &&
    lhs.velY == rhs.velY &&
    lhs.velZ == rhs.velZ &&
    lhs.length == rhs.length &&
    lhs.width == rhs.width &&
    lhs.height == rhs.height;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator1> & lhs, const ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace msg_gen

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator> >
{
  static const char* value()
  {
    return "970207e64b69dddbef9539904dc21d2e";
  }

  static const char* value(const ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x970207e64b69dddbULL;
  static const uint64_t static_value2 = 0xef9539904dc21d2eULL;
};

template<class ContainerAllocator>
struct DataType< ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator> >
{
  static const char* value()
  {
    return "msg_gen/SimOne_Data_Obstacle_Entry";
  }

  static const char* value(const ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 id\n"
"int32 viewId\n"
"SimOne_Obstacle_Type type\n"
"float64 theta\n"
"float64 posX\n"
"float64 posY\n"
"float64 posZ\n"
"float64 oriX\n"
"float64 oriY\n"
"float64 oriZ\n"
"float64 velX\n"
"float64 velY\n"
"float64 velZ\n"
"float64 length\n"
"float64 width\n"
"float64 height\n"
"================================================================================\n"
"MSG: msg_gen/SimOne_Obstacle_Type\n"
"uint8 ESimOne_Obstacle_Type_Unknown = 0\n"
"uint8 ESimOne_Obstacle_Type_Pedestrian = 4\n"
"uint8 ESimOne_Obstacle_Type_Pole = 5\n"
"uint8 ESimOne_Obstacle_Type_Car = 6\n"
"uint8 ESimOne_Obstacle_Type_Static = 7\n"
"uint8 ESimOne_Obstacle_Type_Bicycle = 8\n"
"uint8 ESimOne_Obstacle_Type_Fence = 9\n"
"uint8 ESimOne_Obstacle_Type_RoadMark = 12\n"
"uint8 ESimOne_Obstacle_Type_TrafficSign = 13\n"
"uint8 ESimOne_Obstacle_Type_TrafficLight = 15\n"
"uint8 ESimOne_Obstacle_Type_Rider = 17\n"
"uint8 ESimOne_Obstacle_Type_Truck = 18\n"
"uint8 ESimOne_Obstacle_Type_Bus = 19\n"
"uint8 ESimOne_Obstacle_Type_SpecialVehicle = 20\n"
"uint8 ESimOne_Obstacle_Type_Motorcycle = 21\n"
"uint8 ESimOne_Obstacle_Type_Dynamic = 22\n"
"uint8 ESimOne_Obstacle_Type_GuardRail = 23\n"
"uint8 ESimOne_Obstacle_Type_SpeedLimitSign = 26\n"
"uint8 ESimOne_Obstacle_Type_BicycleStatic = 27\n"
"uint8 ESimOne_Obstacle_Type_RoadObstacle = 29\n"
"uint32 SimOne_Obstacle_Type\n"
;
  }

  static const char* value(const ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.viewId);
      stream.next(m.type);
      stream.next(m.theta);
      stream.next(m.posX);
      stream.next(m.posY);
      stream.next(m.posZ);
      stream.next(m.oriX);
      stream.next(m.oriY);
      stream.next(m.oriZ);
      stream.next(m.velX);
      stream.next(m.velY);
      stream.next(m.velZ);
      stream.next(m.length);
      stream.next(m.width);
      stream.next(m.height);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SimOne_Data_Obstacle_Entry_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::msg_gen::SimOne_Data_Obstacle_Entry_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
    s << indent << "viewId: ";
    Printer<int32_t>::stream(s, indent + "  ", v.viewId);
    s << indent << "type: ";
    s << std::endl;
    Printer< ::msg_gen::SimOne_Obstacle_Type_<ContainerAllocator> >::stream(s, indent + "  ", v.type);
    s << indent << "theta: ";
    Printer<double>::stream(s, indent + "  ", v.theta);
    s << indent << "posX: ";
    Printer<double>::stream(s, indent + "  ", v.posX);
    s << indent << "posY: ";
    Printer<double>::stream(s, indent + "  ", v.posY);
    s << indent << "posZ: ";
    Printer<double>::stream(s, indent + "  ", v.posZ);
    s << indent << "oriX: ";
    Printer<double>::stream(s, indent + "  ", v.oriX);
    s << indent << "oriY: ";
    Printer<double>::stream(s, indent + "  ", v.oriY);
    s << indent << "oriZ: ";
    Printer<double>::stream(s, indent + "  ", v.oriZ);
    s << indent << "velX: ";
    Printer<double>::stream(s, indent + "  ", v.velX);
    s << indent << "velY: ";
    Printer<double>::stream(s, indent + "  ", v.velY);
    s << indent << "velZ: ";
    Printer<double>::stream(s, indent + "  ", v.velZ);
    s << indent << "length: ";
    Printer<double>::stream(s, indent + "  ", v.length);
    s << indent << "width: ";
    Printer<double>::stream(s, indent + "  ", v.width);
    s << indent << "height: ";
    Printer<double>::stream(s, indent + "  ", v.height);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MSG_GEN_MESSAGE_SIMONE_DATA_OBSTACLE_ENTRY_H
