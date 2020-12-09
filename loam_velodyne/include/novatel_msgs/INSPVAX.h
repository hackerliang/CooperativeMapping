// Generated by gencpp from file novatel_msgs/INSPVAX.msg
// DO NOT EDIT!


#ifndef NOVATEL_MSGS_MESSAGE_INSPVAX_H
#define NOVATEL_MSGS_MESSAGE_INSPVAX_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <novatel_msgs/CommonHeader.h>

namespace novatel_msgs
{
template <class ContainerAllocator>
struct INSPVAX_
{
  typedef INSPVAX_<ContainerAllocator> Type;

  INSPVAX_()
    : header()
    , ins_status(0)
    , position_type(0)
    , latitude(0.0)
    , longitude(0.0)
    , altitude(0.0)
    , undulation(0.0)
    , north_velocity(0.0)
    , east_velocity(0.0)
    , up_velocity(0.0)
    , roll(0.0)
    , pitch(0.0)
    , azimuth(0.0)
    , latitude_std(0.0)
    , longitude_std(0.0)
    , altitude_std(0.0)
    , north_velocity_std(0.0)
    , east_velocity_std(0.0)
    , up_velocity_std(0.0)
    , roll_std(0.0)
    , pitch_std(0.0)
    , azimuth_std(0.0)
    , extended_status(0)
    , seconds_since_update(0)  {
    }
  INSPVAX_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , ins_status(0)
    , position_type(0)
    , latitude(0.0)
    , longitude(0.0)
    , altitude(0.0)
    , undulation(0.0)
    , north_velocity(0.0)
    , east_velocity(0.0)
    , up_velocity(0.0)
    , roll(0.0)
    , pitch(0.0)
    , azimuth(0.0)
    , latitude_std(0.0)
    , longitude_std(0.0)
    , altitude_std(0.0)
    , north_velocity_std(0.0)
    , east_velocity_std(0.0)
    , up_velocity_std(0.0)
    , roll_std(0.0)
    , pitch_std(0.0)
    , azimuth_std(0.0)
    , extended_status(0)
    , seconds_since_update(0)  {
  (void)_alloc;
    }



   typedef  ::novatel_msgs::CommonHeader_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint32_t _ins_status_type;
  _ins_status_type ins_status;

   typedef uint32_t _position_type_type;
  _position_type_type position_type;

   typedef double _latitude_type;
  _latitude_type latitude;

   typedef double _longitude_type;
  _longitude_type longitude;

   typedef double _altitude_type;
  _altitude_type altitude;

   typedef float _undulation_type;
  _undulation_type undulation;

   typedef double _north_velocity_type;
  _north_velocity_type north_velocity;

   typedef double _east_velocity_type;
  _east_velocity_type east_velocity;

   typedef double _up_velocity_type;
  _up_velocity_type up_velocity;

   typedef double _roll_type;
  _roll_type roll;

   typedef double _pitch_type;
  _pitch_type pitch;

   typedef double _azimuth_type;
  _azimuth_type azimuth;

   typedef float _latitude_std_type;
  _latitude_std_type latitude_std;

   typedef float _longitude_std_type;
  _longitude_std_type longitude_std;

   typedef float _altitude_std_type;
  _altitude_std_type altitude_std;

   typedef float _north_velocity_std_type;
  _north_velocity_std_type north_velocity_std;

   typedef float _east_velocity_std_type;
  _east_velocity_std_type east_velocity_std;

   typedef float _up_velocity_std_type;
  _up_velocity_std_type up_velocity_std;

   typedef float _roll_std_type;
  _roll_std_type roll_std;

   typedef float _pitch_std_type;
  _pitch_std_type pitch_std;

   typedef float _azimuth_std_type;
  _azimuth_std_type azimuth_std;

   typedef uint32_t _extended_status_type;
  _extended_status_type extended_status;

   typedef uint16_t _seconds_since_update_type;
  _seconds_since_update_type seconds_since_update;


    enum { INS_STATUS_INACTIVE = 0u };
     enum { INS_STATUS_ALIGNING = 1u };
     enum { INS_STATUS_HIGH_VARIANCE = 2u };
     enum { INS_STATUS_SOLUTION_GOOD = 3u };
     enum { INS_STATUS_SOLUTION_FREE = 6u };
     enum { INS_STATUS_ALIGNMENT_COMPLETE = 7u };
     enum { INS_STATUS_DETERMINING_ORIENTATION = 8u };
     enum { INS_STATUS_WAITING_INITIALPOS = 9u };
     enum { POSITION_TYPE_NONE = 0u };
     enum { POSITION_TYPE_SBAS = 52u };
     enum { POSITION_TYPE_PSEUDORANGE_SINGLE_POINT = 53u };
     enum { POSITION_TYPE_PSEUDORANGE_DIFFERENTIAL = 54u };
     enum { POSITION_TYPE_RTK_FLOAT = 55u };
     enum { POSITION_TYPE_RTK_FIXED = 56u };
     enum { POSITION_TYPE_OMNISTAR = 57u };
     enum { POSITION_TYPE_OMNISTAR_HP = 58u };
     enum { POSITION_TYPE_OMNISTAR_XP = 59u };
     enum { POSITION_TYPE_PPP_CONVERGING = 73u };
     enum { POSITION_TYPE_PPP = 74u };
     enum { EXTENDED_STATUS_POSITION_UPDATE_APPLIED = 1u };
     enum { EXTENDED_STATUS_PHASE_UPDATE_APPLIED = 2u };
     enum { EXTENDED_STATUS_ZUPT_APPLIED = 4u };
     enum { EXTENDED_STATUS_WHEEL_SENSOR_APPLIED = 8u };
     enum { EXTENDED_STATUS_HEADING_UPDATE_APPLIED = 16u };
     enum { EXTENDED_STATUS_INS_SOLUTION_CONVERGED = 64u };
 

  typedef boost::shared_ptr< ::novatel_msgs::INSPVAX_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::novatel_msgs::INSPVAX_<ContainerAllocator> const> ConstPtr;

}; // struct INSPVAX_

typedef ::novatel_msgs::INSPVAX_<std::allocator<void> > INSPVAX;

typedef boost::shared_ptr< ::novatel_msgs::INSPVAX > INSPVAXPtr;
typedef boost::shared_ptr< ::novatel_msgs::INSPVAX const> INSPVAXConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::novatel_msgs::INSPVAX_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::novatel_msgs::INSPVAX_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace novatel_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/kinetic/share/nav_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'novatel_msgs': ['/home/slam/catkin_velodyne/src/novatel_span_driver/novatel_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::novatel_msgs::INSPVAX_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::novatel_msgs::INSPVAX_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::novatel_msgs::INSPVAX_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::novatel_msgs::INSPVAX_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::novatel_msgs::INSPVAX_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::novatel_msgs::INSPVAX_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::novatel_msgs::INSPVAX_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b5d66747957184042a6cca9b7368742f";
  }

  static const char* value(const ::novatel_msgs::INSPVAX_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb5d6674795718404ULL;
  static const uint64_t static_value2 = 0x2a6cca9b7368742fULL;
};

template<class ContainerAllocator>
struct DataType< ::novatel_msgs::INSPVAX_<ContainerAllocator> >
{
  static const char* value()
  {
    return "novatel_msgs/INSPVAX";
  }

  static const char* value(const ::novatel_msgs::INSPVAX_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::novatel_msgs::INSPVAX_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# message 1465\n\
novatel_msgs/CommonHeader header\n\
\n\
# Table 29 in the SPAN on OEM6 manual:\n\
# See: http://www.novatel.com/assets/Documents/Manuals/OM-20000144UM.pdf#page=121\n\
uint32 ins_status\n\
uint32 INS_STATUS_INACTIVE=0\n\
uint32 INS_STATUS_ALIGNING=1\n\
uint32 INS_STATUS_HIGH_VARIANCE=2\n\
uint32 INS_STATUS_SOLUTION_GOOD=3\n\
uint32 INS_STATUS_SOLUTION_FREE=6\n\
uint32 INS_STATUS_ALIGNMENT_COMPLETE=7\n\
uint32 INS_STATUS_DETERMINING_ORIENTATION=8\n\
uint32 INS_STATUS_WAITING_INITIALPOS=9\n\
\n\
# Table 30 in the SPAN on OEM6 manual:\n\
# See: http://www.novatel.com/assets/Documents/Manuals/OM-20000144UM.pdf#page=124\n\
uint32 position_type\n\
uint32 POSITION_TYPE_NONE=0\n\
uint32 POSITION_TYPE_SBAS=52\n\
uint32 POSITION_TYPE_PSEUDORANGE_SINGLE_POINT=53\n\
uint32 POSITION_TYPE_PSEUDORANGE_DIFFERENTIAL=54\n\
uint32 POSITION_TYPE_RTK_FLOAT=55\n\
uint32 POSITION_TYPE_RTK_FIXED=56\n\
uint32 POSITION_TYPE_OMNISTAR=57\n\
uint32 POSITION_TYPE_OMNISTAR_HP=58\n\
uint32 POSITION_TYPE_OMNISTAR_XP=59\n\
uint32 POSITION_TYPE_PPP_CONVERGING=73\n\
uint32 POSITION_TYPE_PPP=74\n\
\n\
float64 latitude\n\
float64 longitude\n\
float64 altitude\n\
\n\
float32 undulation\n\
\n\
float64 north_velocity\n\
float64 east_velocity\n\
float64 up_velocity\n\
\n\
float64 roll\n\
float64 pitch\n\
float64 azimuth\n\
\n\
float32 latitude_std\n\
float32 longitude_std\n\
float32 altitude_std\n\
\n\
float32 north_velocity_std\n\
float32 east_velocity_std\n\
float32 up_velocity_std\n\
\n\
float32 roll_std\n\
float32 pitch_std\n\
float32 azimuth_std\n\
\n\
uint32 extended_status\n\
uint32 EXTENDED_STATUS_POSITION_UPDATE_APPLIED=1\n\
uint32 EXTENDED_STATUS_PHASE_UPDATE_APPLIED=2\n\
uint32 EXTENDED_STATUS_ZUPT_APPLIED=4\n\
uint32 EXTENDED_STATUS_WHEEL_SENSOR_APPLIED=8\n\
uint32 EXTENDED_STATUS_HEADING_UPDATE_APPLIED=16\n\
uint32 EXTENDED_STATUS_INS_SOLUTION_CONVERGED=64\n\
\n\
uint16 seconds_since_update\n\
\n\
================================================================================\n\
MSG: novatel_msgs/CommonHeader\n\
# On the wire, this header is preceeded by three sync bytes,\n\
# which are 0xAA 0x44 0x12, and a uint8 which is the header length.\n\
\n\
# Message ID of the log being output.\n\
uint16 id\n\
\n\
# Measurement source, format, response bit.\n\
uint8 msg_type\n\
\n\
uint8 port_addr\n\
uint16 length\n\
uint16 sequence\n\
\n\
uint8 idle_time\n\
uint8 time_status\n\
\n\
uint16 gps_week\n\
uint32 gps_week_seconds\n\
\n\
# Table 3 in the SPAN on OEM6 manual.\n\
# See: http://www.novatel.com/assets/Documents/Manuals/OM-20000144UM.pdf#page=13\n\
uint32 receiver_status\n\
uint32 RECEIVER_STATUS_ERROR=1\n\
uint32 RECEIVER_STATUS_TEMPERATURE_WARNING=2\n\
uint32 RECEIVER_STATUS_VOLTAGE_SUPPLY_WARNING=4\n\
uint32 RECEIVER_STATUS_ANTENNA_UNPOWERED=8\n\
uint32 RECEIVER_STATUS_LNA_FAILURE=16\n\
uint32 RECEIVER_STATUS_ANTENNA_OPEN=32\n\
uint32 RECEIVER_STATUS_ANTENNA_SHORTED=64\n\
uint32 RECEIVER_STATUS_CPU_OVERLOADED=128\n\
uint32 RECEIVER_STATUS_COM1_BUFFER_OVERRUN=256\n\
uint32 RECEIVER_STATUS_COM2_BUFFER_OVERRUN=512\n\
uint32 RECEIVER_STATUS_COM3_BUFFER_OVERRUN=1024\n\
uint32 RECEIVER_STATUS_LINK_OVERLOAD=2048\n\
uint32 RECEIVER_STATUS_AUX_TRANSMIT_OVERRUN=8192\n\
uint32 RECEIVER_STATUS_AGC_OUT_OF_RANGE=16384\n\
uint32 RECEIVER_STATUS_INS_RESET=65536\n\
uint32 RECEIVER_STATUS_ALMANAC_INVALID=262144\n\
uint32 RECEIVER_STATUS_POSITION_SOLUTION_INVALID=524288\n\
uint32 RECEIVER_STATUS_POSITION_NOT_FIXED=1048576\n\
uint32 RECEIVER_STATUS_CLOCK_STEERING_DISABLED=2097152\n\
uint32 RECEIVER_STATUS_CLOCK_MODEL_INVALID=4194304\n\
uint32 RECEIVER_STATUS_EXTERNAL_OSCILLATOR_LOCKED=8388608\n\
uint32 RECEIVER_STATUS_SOFTWARE_RESOURCE_WARNING=16777216\n\
uint32 RECEIVER_STATUS_AUXILIARY3_EVENT=536870912\n\
uint32 RECEIVER_STATUS_AUXILIARY2_EVENT=1073741824\n\
uint32 RECEIVER_STATUS_AUXILIARY1_EVENT=2147483648\n\
\n\
uint16 reserved\n\
uint16 software_version\n\
\n\
";
  }

  static const char* value(const ::novatel_msgs::INSPVAX_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::novatel_msgs::INSPVAX_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.ins_status);
      stream.next(m.position_type);
      stream.next(m.latitude);
      stream.next(m.longitude);
      stream.next(m.altitude);
      stream.next(m.undulation);
      stream.next(m.north_velocity);
      stream.next(m.east_velocity);
      stream.next(m.up_velocity);
      stream.next(m.roll);
      stream.next(m.pitch);
      stream.next(m.azimuth);
      stream.next(m.latitude_std);
      stream.next(m.longitude_std);
      stream.next(m.altitude_std);
      stream.next(m.north_velocity_std);
      stream.next(m.east_velocity_std);
      stream.next(m.up_velocity_std);
      stream.next(m.roll_std);
      stream.next(m.pitch_std);
      stream.next(m.azimuth_std);
      stream.next(m.extended_status);
      stream.next(m.seconds_since_update);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct INSPVAX_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::novatel_msgs::INSPVAX_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::novatel_msgs::INSPVAX_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::novatel_msgs::CommonHeader_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "ins_status: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.ins_status);
    s << indent << "position_type: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.position_type);
    s << indent << "latitude: ";
    Printer<double>::stream(s, indent + "  ", v.latitude);
    s << indent << "longitude: ";
    Printer<double>::stream(s, indent + "  ", v.longitude);
    s << indent << "altitude: ";
    Printer<double>::stream(s, indent + "  ", v.altitude);
    s << indent << "undulation: ";
    Printer<float>::stream(s, indent + "  ", v.undulation);
    s << indent << "north_velocity: ";
    Printer<double>::stream(s, indent + "  ", v.north_velocity);
    s << indent << "east_velocity: ";
    Printer<double>::stream(s, indent + "  ", v.east_velocity);
    s << indent << "up_velocity: ";
    Printer<double>::stream(s, indent + "  ", v.up_velocity);
    s << indent << "roll: ";
    Printer<double>::stream(s, indent + "  ", v.roll);
    s << indent << "pitch: ";
    Printer<double>::stream(s, indent + "  ", v.pitch);
    s << indent << "azimuth: ";
    Printer<double>::stream(s, indent + "  ", v.azimuth);
    s << indent << "latitude_std: ";
    Printer<float>::stream(s, indent + "  ", v.latitude_std);
    s << indent << "longitude_std: ";
    Printer<float>::stream(s, indent + "  ", v.longitude_std);
    s << indent << "altitude_std: ";
    Printer<float>::stream(s, indent + "  ", v.altitude_std);
    s << indent << "north_velocity_std: ";
    Printer<float>::stream(s, indent + "  ", v.north_velocity_std);
    s << indent << "east_velocity_std: ";
    Printer<float>::stream(s, indent + "  ", v.east_velocity_std);
    s << indent << "up_velocity_std: ";
    Printer<float>::stream(s, indent + "  ", v.up_velocity_std);
    s << indent << "roll_std: ";
    Printer<float>::stream(s, indent + "  ", v.roll_std);
    s << indent << "pitch_std: ";
    Printer<float>::stream(s, indent + "  ", v.pitch_std);
    s << indent << "azimuth_std: ";
    Printer<float>::stream(s, indent + "  ", v.azimuth_std);
    s << indent << "extended_status: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.extended_status);
    s << indent << "seconds_since_update: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.seconds_since_update);
  }
};

} // namespace message_operations
} // namespace ros

#endif // NOVATEL_MSGS_MESSAGE_INSPVAX_H
