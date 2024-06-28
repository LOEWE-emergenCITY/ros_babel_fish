// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <ros_babel_fish/babel_fish.hpp>
#include <ros_babel_fish/macros.hpp>
#include <thread>
#include <utility>

/*!
 * The following example demonstrates how to subscribe to a topic of any type,
 *   retrieve the topic's message type, and traverse the message's content.
 */

void dumpMessageContent( const ros_babel_fish::Message &message, const std::string &prefix = "" );

using std::placeholders::_1;

class AnySubscriber : public rclcpp::Node
{
public:
  explicit AnySubscriber( std::string topic )
      : Node( "any_subscriber" ), topic_( std::move( topic ) )
  {
  }

  void init()
  {
    fish = ros_babel_fish::BabelFish::make_shared();
    subscription_ = fish->create_subscription(
        *this, topic_, 10,
        [this]( ros_babel_fish::CompoundMessage::SharedPtr msg ) { topic_callback( msg ); } );
  }

private:
  void topic_callback( const ros_babel_fish::CompoundMessage::SharedPtr &msg ) const
  {
    if ( msg == nullptr ) {
      RCLCPP_INFO( this->get_logger(), "I heard a null message" );
      return;
    }
    try {
      std::cout << "Type: " << msg->name() << std::endl;
      dumpMessageContent( *msg );
      std::cout << std::endl << "---" << std::endl;
    } catch ( ros_babel_fish::BabelFishException &ex ) {
      RCLCPP_ERROR( this->get_logger(), "Got a BabelFishException during translation: %s", ex.what() );
    }
  }

  std::string topic_;
  ros_babel_fish::BabelFish::SharedPtr fish;
  ros_babel_fish::BabelFishSubscription::SharedPtr subscription_;
};

int main( int argc, char *argv[] )
{
  if ( argc != 2 ) {
    std::cout << "Invalid argument!" << std::endl;
    std::cout << "Usage: any_subscriber [TOPIC]" << std::endl;
    return 1;
  }
  rclcpp::init( argc, argv );
  auto node = std::make_shared<AnySubscriber>( argv[1] );
  node->init();
  rclcpp::spin( node );
  rclcpp::shutdown();
  return 0;
}

// Here's where the dumping happens

template<typename T>
void printToStdOut( const T &val )
{
  std::cout << val;
}

template<typename T>
void printValueMessageToStdOut( const ros_babel_fish::Message &val )
{
  printToStdOut( val.value<T>() );
}

// Specialize for types that don't support stream operator or where we wan't to handle it differently
template<>
void printToStdOut<std::wstring>( const std::wstring &val )
{
  std::wcout << val; // Need wide cout for wide string
}

template<>
void printToStdOut<uint8_t>( const uint8_t &val )
{
  std::cout << static_cast<int>( val ); // Cast to int for proper displaying
}

template<typename T>
void printArray( const T &message, const std::string & )
{
  std::cout << "[";
  for ( size_t i = 0; i < message.size(); ++i ) {
    printToStdOut( message[i] );
    if ( i != message.size() - 1 )
      std::cout << ", ";
  }
  std::cout << "]" << std::endl;
}

template<bool BOUNDED, bool FIXED_LENGTH>
void printArray( const ros_babel_fish::CompoundArrayMessage_<BOUNDED, FIXED_LENGTH> &message,
                 const std::string &prefix )
{
  std::cout << std::endl;
  for ( size_t i = 0; i < message.size(); ++i ) {
    std::cout << prefix << "- ";
    dumpMessageContent( message[i], prefix + "  " );
  }
}

// Casts the ArrayMessageBase and calls the actual printArray
template<typename T>
void printArrayMessage( const ros_babel_fish::ArrayMessageBase &message, const std::string &prefix )
{
  printArray( message.as<ros_babel_fish::ArrayMessage<T>>(), prefix );
}

void dumpMessageContent( const ros_babel_fish::Message &message, const std::string &prefix )
{
  using namespace ros_babel_fish;
  if ( message.type() == MessageTypes::Compound ) {
    auto &compound = message.as<CompoundMessage>();
    // We can either just dump the message or demonstrate the time conversion
    if ( compound.isTime() ) {
      rclcpp::Time time = compound.value<rclcpp::Time>();
      std::cout << "Time { seconds: " << time.seconds() << " }";
      return;
    }
    if ( compound.isDuration() ) {
      auto duration = compound.value<rclcpp::Duration>();
      std::cout << "Duration { seconds: " << duration.seconds() << " }";
      return;
    }
    std::cout << std::endl;
    for ( size_t i = 0; i < compound.keys().size(); ++i ) {
      std::cout << prefix << compound.keys()[i] << ": ";
      dumpMessageContent( *compound.values()[i], prefix + "  " );
      if ( i != compound.keys().size() - 1 )
        std::cout << std::endl;
    }
  } else if ( message.type() == MessageTypes::Array ) {
    auto &base = message.as<ArrayMessageBase>();
    RBF2_TEMPLATE_CALL_ARRAY_TYPES( printArray, base, prefix );
  } else {
    switch ( message.type() ) {
    case MessageTypes::Array:
    case MessageTypes::Compound:
    case MessageTypes::None:
      break;
    case MessageTypes::Bool:
      std::cout << ( message.as<ValueMessage<bool>>().getValue() ? "true" : "false" );
      break;
    case MessageTypes::UInt8:
      std::cout << static_cast<unsigned int>( message.as<ValueMessage<uint8_t>>().getValue() );
      break;
    case MessageTypes::UInt16:
      std::cout << message.value<
          uint16_t>(); // The statement above can be simplified using this convenience method
      break;
    default:
      // Alternatively you could replace the whole switch with the following macro from macros.h
      RBF2_TEMPLATE_CALL_VALUE_TYPES( printValueMessageToStdOut, message.type(), message );
      // Or use one of the invoke methods in template_helpers.hpp for a better C++ solution that can
      // also handle functors with a return value.
    }
  }
}
