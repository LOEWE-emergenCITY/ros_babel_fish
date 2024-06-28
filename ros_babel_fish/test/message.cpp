//
// Created by Stefan Fabian on 07.09.19.
//

#include "message_comparison.hpp"

#include <ros_babel_fish/babel_fish.hpp>
#include <ros_babel_fish/idl/providers/local_type_support_provider.hpp>
#include <ros_babel_fish/idl/serialization.hpp>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

using namespace ros_babel_fish;
using namespace std::chrono_literals;

template<typename T>
::testing::AssertionResult compareArrays( const T *a, const T *b, size_t count )
{
  for ( size_t i = 0; i < count; ++i ) {
    if ( a[i] == b[i] )
      continue;
    return ::testing::AssertionFailure() << "Difference at index " << i << ":" << std::endl
                                         << "a[[" << i << "]: " << a[i] << std::endl
                                         << "b[" << i << "]: " << b[i];
  }
  return ::testing::AssertionSuccess();
}

template<typename T1, typename T2>
::testing::AssertionResult compareVectors( const T1 &a, const T2 &b, size_t count )
{
  for ( size_t i = 0; i < count; ++i ) {
    if ( a[i] == b[i] )
      continue;
    return ::testing::AssertionFailure() << "Difference at index " << i << ":" << std::endl
                                         << "a[[" << i << "]: " << a[i] << std::endl
                                         << "b[" << i << "]: " << b[i];
  }
  return ::testing::AssertionSuccess();
}

std::shared_ptr<void> createSharedWrapper( void *data )
{
  return std::shared_ptr<void>( data, []( auto * ) { /* No ownership, so no delete */ } );
}

TEST( MessageTest, message )
{
  rosidl_typesupport_introspection_cpp::MessageMember member;
  MessageMemberIntrospection introspection(
      &member, nullptr ); // Wrapper since we don't load from a library here
  member.offset_ = 0;
  // UINT8
  {
    member.type_id_ = MessageTypes::UInt8;
    uint8_t data = 42;
    ValueMessage<uint8_t> vm_in( introspection, createSharedWrapper( &data ) );
    Message &vm = vm_in;
    EXPECT_THROW( vm["invalid"], BabelFishException );
    const Message &vmc = vm;
    EXPECT_THROW( vmc["answer"], BabelFishException );
    EXPECT_EQ( vm.value<uint8_t>(), 42U );
    EXPECT_NO_THROW( vm = uint8_t( 55 ) );
    EXPECT_EQ( vmc.value<int32_t>(), 55 );
    EXPECT_THROW( vm = true, BabelFishException );

    // This is within the limits of the type and should trigger a warning at most but not throw
    EXPECT_NO_THROW( vm = 255 );
    EXPECT_EQ( vm.value<uint8_t>(), 255 );
    // This is not within the limits of the type and should throw
    EXPECT_THROW( vm = 256, BabelFishException );

    EXPECT_THROW( vm = true, BabelFishException );
    EXPECT_THROW( vm = rclcpp::Time( 42 ), BabelFishException );
    EXPECT_THROW( vm = rclcpp::Duration( 42ns ), BabelFishException );
    EXPECT_THROW( vm.value<std::string>(), BabelFishException );
    EXPECT_THROW( vm.value<rclcpp::Duration>(), BabelFishException );
    EXPECT_THROW( vm.value<bool>(), BabelFishException );
    EXPECT_EQ( data, 255U );
  }

  // UINT16
  {
    member.type_id_ = MessageTypes::UInt16;
    uint16_t data = 42;
    ValueMessage<uint16_t> vm_in( introspection, createSharedWrapper( &data ) );
    Message &vm = vm_in;
    EXPECT_EQ( vm.value<uint16_t>(), 42U );
    EXPECT_NO_THROW( vm = uint16_t( 355 ) );
    EXPECT_EQ( vm.value<int32_t>(), 355 );
    EXPECT_EQ( data, 355U );
  }

  // UINT32
  {
    member.type_id_ = MessageTypes::UInt32;
    uint32_t data = 42;
    ValueMessage<uint32_t> vm_in( introspection, createSharedWrapper( &data ) );
    Message &vm = vm_in;
    EXPECT_EQ( vm.value<uint32_t>(), 42U );
    EXPECT_NO_THROW( vm = uint32_t( 133755 ) );
    EXPECT_EQ( vm.value<int64_t>(), 133755 );
    EXPECT_EQ( data, 133755U );
  }

  // UINT64
  {
    member.type_id_ = MessageTypes::UInt64;
    uint64_t data = 42;
    ValueMessage<uint64_t> vm_in( introspection, createSharedWrapper( &data ) );
    Message &vm = vm_in;
    EXPECT_EQ( vm.value<uint64_t>(), 42U );
    EXPECT_NO_THROW( vm = uint64_t( 2 ) << uint64_t( 34U ) );
    EXPECT_EQ( vm.value<int64_t>(), int64_t( 2 ) << int64_t( 34 ) );
    EXPECT_EQ( data, uint64_t( 2 ) << uint64_t( 34 ) );
  }

  // INT8
  {
    member.type_id_ = MessageTypes::Int8;
    int8_t data = 42;
    ValueMessage<int8_t> vm_in( introspection, createSharedWrapper( &data ) );
    Message &vm = vm_in;
    EXPECT_THROW( vm["invalid"], BabelFishException );
    const Message &vmc = vm;
    EXPECT_THROW( vmc["answer"], BabelFishException );
    EXPECT_EQ( vm.value<int8_t>(), 42 );
    EXPECT_NO_THROW( vm = int8_t( 55 ) );
    EXPECT_EQ( vmc.value<int32_t>(), 55 );
    EXPECT_THROW( vm = true, BabelFishException );

    // This is within the limits of the type and should trigger a warning at most but not throw
    EXPECT_NO_THROW( vm = 127 );
    EXPECT_EQ( vm.value<int8_t>(), 127 );
    // This is not within the limits of the type and should throw
    EXPECT_THROW( vm = 128, BabelFishException );
    EXPECT_THROW( vm = -129, BabelFishException );

    EXPECT_THROW( vm = true, BabelFishException );
    EXPECT_THROW( vm = rclcpp::Time( 42 ), BabelFishException );
    EXPECT_THROW( vm = rclcpp::Duration( 42ns ), BabelFishException );
    EXPECT_EQ( data, 127 );
  }

  // INT16
  {
    member.type_id_ = MessageTypes::Int16;
    int16_t data = 42;
    ValueMessage<int16_t> vm_in( introspection, createSharedWrapper( &data ) );
    Message &vm = vm_in;
    EXPECT_EQ( vm.value<int16_t>(), 42 );
    EXPECT_NO_THROW( vm = int16_t( 129 ) );
    EXPECT_EQ( vm.value<int32_t>(), 129 );
    EXPECT_EQ( data, 129 );
  }

  // INT32
  {
    member.type_id_ = MessageTypes::Int32;
    int32_t data = 42;
    ValueMessage<int32_t> vm_in( introspection, createSharedWrapper( &data ) );
    Message &vm = vm_in;
    EXPECT_EQ( vm.value<int32_t>(), 42 );
    EXPECT_NO_THROW( vm = int32_t( 70501 ) );
    EXPECT_EQ( vm.value<int32_t>(), 70501 );
    EXPECT_EQ( data, 70501 );
  }

  // INT64
  {
    member.type_id_ = MessageTypes::Int64;
    int64_t data = 42;
    ValueMessage<int64_t> vm_in( introspection, createSharedWrapper( &data ) );
    Message &vm = vm_in;
    EXPECT_EQ( vm.value<int64_t>(), 42 );
    EXPECT_NO_THROW( vm = static_cast<int64_t>( int64_t( 2 ) << int64_t( 34 ) ) );
    EXPECT_EQ( vm.value<int64_t>(), int64_t( 2 ) << int64_t( 34 ) );
    EXPECT_THROW( vm.value<int32_t>(), BabelFishException );
    EXPECT_EQ( data, int64_t( 2 ) << int64_t( 34 ) );
  }

  // BOOL
  {
    member.type_id_ = MessageTypes::Bool;
    bool data = false;
    ValueMessage<bool> vm_in( introspection, createSharedWrapper( &data ) );
    Message &vm = vm_in;
    EXPECT_FALSE( vm.value<bool>() );
    EXPECT_THROW( vm.value<int32_t>(), BabelFishException );
    EXPECT_NO_THROW( vm = true );
    EXPECT_TRUE( vm.value<bool>() );
    EXPECT_THROW( vm = 42, BabelFishException );
    EXPECT_EQ( data, true );
  }

  // FLOAT
  {
    member.type_id_ = MessageTypes::Float;
    float data = 42.0;
    ValueMessage<float> vm_in( introspection, createSharedWrapper( &data ) );
    Message &vm = vm_in;
    EXPECT_FLOAT_EQ( vm.value<float>(), 42.0f );
    ASSERT_NO_THROW( vm.value<double>() );
    EXPECT_DOUBLE_EQ( vm.value<double>(), 42.0 );
    EXPECT_NO_THROW( vm = 50.0f );
    EXPECT_FLOAT_EQ( vm.value<float>(), 50.0f );
    EXPECT_THROW( vm = 42.0, BabelFishException );
    EXPECT_FLOAT_EQ( vm.value<float>(), 50.0f );
    EXPECT_NO_THROW( vm = 42 );
    EXPECT_FLOAT_EQ( vm.value<float>(), 42.0f );
    EXPECT_THROW( vm.value<int32_t>(), BabelFishException );
    EXPECT_FLOAT_EQ( data, 42.0f );
  }

  // DOUBLE
  {
    member.type_id_ = MessageTypes::Double;
    double data = 42.0;
    ValueMessage<double> vm_in( introspection, createSharedWrapper( &data ) );
    Message &vm = vm_in;
    EXPECT_DOUBLE_EQ( vm.value<double>(), 42.0 );
    EXPECT_THROW( vm.value<float>(), BabelFishException );
    EXPECT_NO_THROW( vm = 50.0f );
    EXPECT_DOUBLE_EQ( vm.value<double>(), 50.0 );
    EXPECT_NO_THROW( vm = 42.0 );
    EXPECT_DOUBLE_EQ( vm.value<double>(), 42.0 );
    EXPECT_NO_THROW( vm = 50 );
    EXPECT_DOUBLE_EQ( vm.value<double>(), 50.0 );
    EXPECT_THROW( vm.value<int32_t>(), BabelFishException );
    EXPECT_DOUBLE_EQ( data, 50.0f );
  }

  // STRING
  {
    member.type_id_ = MessageTypes::String;
    std::string data = "test";
    ValueMessage<std::string> vm_in( introspection, createSharedWrapper( &data ) );
    Message &vm = vm_in;
    EXPECT_EQ( vm.value<std::string>(), "test" );
    vm = "the answer";
    EXPECT_NO_THROW( vm = "the answer" );
    EXPECT_EQ( vm.value<std::string>(), "the answer" );
    EXPECT_NO_THROW( vm = std::string( "42" ) );
    EXPECT_EQ( vm.value<std::string>(), "42" );
    EXPECT_THROW( vm.value<int32_t>(), BabelFishException );
    EXPECT_THROW( vm.value<double>(), BabelFishException );
    EXPECT_THROW( vm.value<rclcpp::Time>(), BabelFishException );
    EXPECT_THROW( vm = 12.0, BabelFishException );
    EXPECT_EQ( data, "42" );
  }

  // COMPOUND
  {
    ros_babel_fish::LocalTypeSupportProvider provider;
    MessageTypeSupport::ConstSharedPtr type_support =
        provider.getMessageTypeSupport( "std_msgs/msg/Header" );
    CompoundMessage m( *type_support );
    Message &vm = m;
    EXPECT_THROW( vm = 42, BabelFishException );
  }
}

TEST( MessageTest, compoundMessage )
{
  ros_babel_fish::BabelFish fish;
  MessageTypeSupport::ConstSharedPtr type_support =
      fish.get_message_type_support( "geometry_msgs/msg/PoseStamped" );
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = "test_frame";
  pose_msg.header.stamp = rclcpp::Time( 42L, RCL_ROS_TIME );
  pose_msg.pose.position.x = 1;
  pose_msg.pose.position.y = 2;
  bool shared_pointer_alive = true;
  std::shared_ptr<void> data( &pose_msg,
                              [&shared_pointer_alive]( auto * ) { shared_pointer_alive = false; } );
  auto msg = CompoundMessage::make_unique( *type_support, std::move( data ) );
  ASSERT_TRUE( shared_pointer_alive );

  {
    const auto &header =
        ( *msg )["header"]; // Note that this would not keep the pointer alive since we only have a reference.
    EXPECT_EQ( header["frame_id"], "test_frame" );
    EXPECT_EQ( header["stamp"], rclcpp::Time( 42L, RCL_ROS_TIME ) );
  }
  ASSERT_TRUE( shared_pointer_alive );

  {
    auto pose = ( *msg )["pose"].as<CompoundMessage>();
    msg.reset();
    ASSERT_TRUE( shared_pointer_alive );
    EXPECT_EQ( pose["position"]["x"], 1 );
    EXPECT_EQ( pose["position"]["y"], 2 );
  }

  ASSERT_FALSE( shared_pointer_alive ) << "Data should be destructed after last reference is gone!";
}

TEST( MessageTest, arrayMessage )
{
  // Creation not supported yet
  //  // Compound
  //  {
  //    BabelFish fish;
  //    std::shared_ptr<void> data;
  //    MessageMembersIntrospection introspection(
  //      *fish.get_message_type_support( "std_msgs/msg/Header" ));
  //    CompoundArrayMessage am( introspection.getMember(0), data );
  //    for ( int i = 0; i < 20; ++i )
  //    {
  //      auto &cm = am.appendEmpty();
  //      cm["stamp"] = rclcpp::Time( 20 * i );
  //      cm["frame_id"] = std::string( "frame " ) + std::to_string( i );
  //    }
  //    EXPECT_EQ( am.size(), 20U );
  //    EXPECT_THROW( am.assign( 20, CompoundMessage{} ), BabelFishException );

  //    CompoundArrayMessage am_copy( am.elementIntrospection() );
  //    for ( int i = 0; i < 10; ++i )
  //    {
  //      auto cm = fish.create_message( "std_msgs/msg/Header" );
  //      cm["stamp"] = rclcpp::Time( 200 * i );
  //      cm["frame_id"] = std::string( "copy frame " ) + std::to_string( i );
  //      am_copy.push_back( cm );
  //    }
  //    EXPECT_EQ( am_copy.size(), 10U );
  //    am_copy = am;
  //    EXPECT_EQ( am_copy.elementDatatype(), "std_msgs::msg::Header" );
  //    EXPECT_EQ( am_copy.size(), 20U );

  //    auto *am_clone = dynamic_cast<CompoundArrayMessage *>(am.clone());
  //    EXPECT_EQ( am_clone->size(), 20U );
  //    am_clone->at( 0 )["frame_id"] = "different_frame";
  //    EXPECT_EQ( am_clone->at( 0 )["frame_id"].value<std::string>(), "different_frame" );
  //    EXPECT_EQ( am[0]["frame_id"].value<std::string>(), "frame 0" );
  //    delete am_clone;

  //    CompoundArrayMessage different_am(
  //      fish.getMessageTypeSupport( "geometry_msgs/msg/Pose" )->message_template );
  //    EXPECT_THROW( am_copy = different_am, BabelFishException );

  //    ArrayMessage<Message> aa( MessageTypes::Array );
  //    aa.push_back( am.clone());
  //    aa.push_back( am_copy.clone());
  //    EXPECT_EQ( aa.size(), 2U );
  //    EXPECT_EQ( aa[0].as<CompoundArrayMessage>()[0]["frame_id"].value<std::string>(), "frame 0" );
  //    auto *aa_clone = dynamic_cast<ArrayMessage<Message> *>(aa.clone());
  //    ASSERT_NE( aa_clone, nullptr );
  //    EXPECT_EQ( aa_clone->size(), 2U );
  //    EXPECT_EQ( aa[0].as<CompoundArrayMessage>()[0]["frame_id"].value<std::string>(), "frame 0" );
  //    delete aa_clone;
  //  }

  //  // FIXED SIZE
  //  {
  //    ArrayMessage<bool> am( 20, true );
  //    EXPECT_THROW( am.push_back( true ), BabelFishException );
  //  }
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  return RUN_ALL_TESTS();
}
