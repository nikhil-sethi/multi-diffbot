// Generated by gencpp from file bullproof_nav/NavPose2DResult.msg
// DO NOT EDIT!


#ifndef BULLPROOF_NAV_MESSAGE_NAVPOSE2DRESULT_H
#define BULLPROOF_NAV_MESSAGE_NAVPOSE2DRESULT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace bullproof_nav
{
template <class ContainerAllocator>
struct NavPose2DResult_
{
  typedef NavPose2DResult_<ContainerAllocator> Type;

  NavPose2DResult_()
    : reached(false)  {
    }
  NavPose2DResult_(const ContainerAllocator& _alloc)
    : reached(false)  {
  (void)_alloc;
    }



   typedef uint8_t _reached_type;
  _reached_type reached;





  typedef boost::shared_ptr< ::bullproof_nav::NavPose2DResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bullproof_nav::NavPose2DResult_<ContainerAllocator> const> ConstPtr;

}; // struct NavPose2DResult_

typedef ::bullproof_nav::NavPose2DResult_<std::allocator<void> > NavPose2DResult;

typedef boost::shared_ptr< ::bullproof_nav::NavPose2DResult > NavPose2DResultPtr;
typedef boost::shared_ptr< ::bullproof_nav::NavPose2DResult const> NavPose2DResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bullproof_nav::NavPose2DResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bullproof_nav::NavPose2DResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::bullproof_nav::NavPose2DResult_<ContainerAllocator1> & lhs, const ::bullproof_nav::NavPose2DResult_<ContainerAllocator2> & rhs)
{
  return lhs.reached == rhs.reached;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::bullproof_nav::NavPose2DResult_<ContainerAllocator1> & lhs, const ::bullproof_nav::NavPose2DResult_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace bullproof_nav

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::bullproof_nav::NavPose2DResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bullproof_nav::NavPose2DResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bullproof_nav::NavPose2DResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bullproof_nav::NavPose2DResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bullproof_nav::NavPose2DResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bullproof_nav::NavPose2DResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bullproof_nav::NavPose2DResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "80204a6ff2622a7071680d5597cbd3aa";
  }

  static const char* value(const ::bullproof_nav::NavPose2DResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x80204a6ff2622a70ULL;
  static const uint64_t static_value2 = 0x71680d5597cbd3aaULL;
};

template<class ContainerAllocator>
struct DataType< ::bullproof_nav::NavPose2DResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bullproof_nav/NavPose2DResult";
  }

  static const char* value(const ::bullproof_nav::NavPose2DResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bullproof_nav::NavPose2DResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"bool reached\n"
;
  }

  static const char* value(const ::bullproof_nav::NavPose2DResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bullproof_nav::NavPose2DResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.reached);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NavPose2DResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bullproof_nav::NavPose2DResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bullproof_nav::NavPose2DResult_<ContainerAllocator>& v)
  {
    s << indent << "reached: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reached);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BULLPROOF_NAV_MESSAGE_NAVPOSE2DRESULT_H
