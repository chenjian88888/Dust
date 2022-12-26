// Generated by gencpp from file msg_gen/sensordetections.msg
// DO NOT EDIT!


#ifndef MSG_GEN_MESSAGE_SENSORDETECTIONS_H
#define MSG_GEN_MESSAGE_SENSORDETECTIONS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <msg_gen/SimOne_Data_SensorDetections_Entry.h>

namespace msg_gen
{
template <class ContainerAllocator>
struct sensordetections_
{
  typedef sensordetections_<ContainerAllocator> Type;

  sensordetections_()
    : objectSize(0)
    , objects()  {
    }
  sensordetections_(const ContainerAllocator& _alloc)
    : objectSize(0)
    , objects(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _objectSize_type;
  _objectSize_type objectSize;

   typedef std::vector< ::msg_gen::SimOne_Data_SensorDetections_Entry_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::msg_gen::SimOne_Data_SensorDetections_Entry_<ContainerAllocator> >::other >  _objects_type;
  _objects_type objects;





  typedef boost::shared_ptr< ::msg_gen::sensordetections_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::msg_gen::sensordetections_<ContainerAllocator> const> ConstPtr;

}; // struct sensordetections_

typedef ::msg_gen::sensordetections_<std::allocator<void> > sensordetections;

typedef boost::shared_ptr< ::msg_gen::sensordetections > sensordetectionsPtr;
typedef boost::shared_ptr< ::msg_gen::sensordetections const> sensordetectionsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::msg_gen::sensordetections_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::msg_gen::sensordetections_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::msg_gen::sensordetections_<ContainerAllocator1> & lhs, const ::msg_gen::sensordetections_<ContainerAllocator2> & rhs)
{
  return lhs.objectSize == rhs.objectSize &&
    lhs.objects == rhs.objects;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::msg_gen::sensordetections_<ContainerAllocator1> & lhs, const ::msg_gen::sensordetections_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace msg_gen

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::msg_gen::sensordetections_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msg_gen::sensordetections_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_gen::sensordetections_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_gen::sensordetections_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_gen::sensordetections_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_gen::sensordetections_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::msg_gen::sensordetections_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5e3bf6f1a0360393a21effdeb55e056e";
  }

  static const char* value(const ::msg_gen::sensordetections_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5e3bf6f1a0360393ULL;
  static const uint64_t static_value2 = 0xa21effdeb55e056eULL;
};

template<class ContainerAllocator>
struct DataType< ::msg_gen::sensordetections_<ContainerAllocator> >
{
  static const char* value()
  {
    return "msg_gen/sensordetections";
  }

  static const char* value(const ::msg_gen::sensordetections_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::msg_gen::sensordetections_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 objectSize\n"
"SimOne_Data_SensorDetections_Entry[] objects\n"
"================================================================================\n"
"MSG: msg_gen/SimOne_Data_SensorDetections_Entry\n"
"int32 id\n"
"SimOne_Obstacle_Type type\n"
"float64 posX\n"
"float64 posY\n"
"float64 posZ\n"
"float64 oriX\n"
"float64 oriY\n"
"float64 oriZ\n"
"float64 length\n"
"float64 width\n"
"float64 height\n"
"float64 range\n"
"float64 velX\n"
"float64 velY\n"
"float64 velZ\n"
"float64 probability\n"
"float64 relativePosX\n"
"float64 relativePosY\n"
"float64 relativePosZ\n"
"float64 relativeRotX\n"
"float64 relativeRotY\n"
"float64 relativeRotZ\n"
"float64 relativeVelX\n"
"float64 relativeVelY\n"
"float64 relativeVelZ\n"
"float64 bbox2dMinX\n"
"float64 bbox2dMinY\n"
"float64 bbox2dMaxX\n"
"float64 bbox2dMaxY\n"
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

  static const char* value(const ::msg_gen::sensordetections_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::msg_gen::sensordetections_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.objectSize);
      stream.next(m.objects);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct sensordetections_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::msg_gen::sensordetections_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::msg_gen::sensordetections_<ContainerAllocator>& v)
  {
    s << indent << "objectSize: ";
    Printer<int32_t>::stream(s, indent + "  ", v.objectSize);
    s << indent << "objects[]" << std::endl;
    for (size_t i = 0; i < v.objects.size(); ++i)
    {
      s << indent << "  objects[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::msg_gen::SimOne_Data_SensorDetections_Entry_<ContainerAllocator> >::stream(s, indent + "    ", v.objects[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MSG_GEN_MESSAGE_SENSORDETECTIONS_H
