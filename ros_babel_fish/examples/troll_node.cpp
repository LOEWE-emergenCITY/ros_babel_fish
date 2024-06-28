// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <rclcpp/rclcpp.hpp>
#include <ros_babel_fish/babel_fish.hpp>
#include <ros_babel_fish/messages/array_message.hpp>
#include <ros_babel_fish/messages/compound_message.hpp>
#include <ros_babel_fish/messages/value_message.hpp>
#include <ros_babel_fish/method_invoke_helpers.hpp>

/*!
 * The following example demonstrates how this library can be used to receive messages on one topic, modify elements
 * of the message and republish them on a different topic.
 * In this particular example, the incoming message is searched for string fields and their value is replaced with the
 * lyrics of Rick Astley's Never Gonna Give You Up.
 */

using namespace ros_babel_fish;

void updateMessage( Message &message );

class TrollNode : public rclcpp::Node
{
public:
  TrollNode( std::string in_topic, std::string out_topic )
      : Node( "troll_node" ), in_topic_( std::move( in_topic ) ), out_topic_( std::move( out_topic ) )
  {
  }

  void init()
  {
    // This is in init because we need a shared_ptr from this node and this won't work in the constructor
    sub_ = fish_.create_subscription(
        *this, in_topic_, 1, [this]( CompoundMessage::SharedPtr msg ) { processMessage( msg ); } );
    RCLCPP_INFO( get_logger(), "Initialized. Waiting for messages on %s", in_topic_.c_str() );
  }

  void processMessage( const CompoundMessage::SharedPtr &message )
  {
    if ( pub_ == nullptr ) {
      RCLCPP_INFO( get_logger(), "Received a message. Starting publisher." );
      pub_ = fish_.create_publisher( *this, out_topic_, message->name(), 1 );
      RCLCPP_INFO( get_logger(), "Publisher created." );
    }
    updateMessage( *message );
    pub_->publish( *message );
    RCLCPP_INFO( get_logger(), "Message updated." );
  }

  BabelFish fish_;
  BabelFishSubscription::SharedPtr sub_;
  BabelFishPublisher::SharedPtr pub_;
  std::string in_topic_;
  std::string out_topic_;
};

int main( int argc, char **argv )
{
  if ( argc != 3 ) {
    std::cout << "Invalid argument!" << std::endl;
    std::cout << "Usage: troll_node [INPUT TOPIC] [OUTPUT_TOPIC]" << std::endl;
    return 1;
  }

  rclcpp::init( argc, argv );
  std::string in_topic = argv[1];
  std::string out_topic = argv[2];
  auto node = std::make_shared<TrollNode>( in_topic, out_topic );
  node->init();
  rclcpp::spin( node );
  rclcpp::shutdown();
  return 0;
}

static constexpr int song_text_length = 56;
//! The lyrics of Rick Astley's masterpiece "Never Gonna Give You Up" (written and produced by Stock Aitken Waterman)
std::string song_text[song_text_length] = {
    "We're no strangers to love",
    "You know the rules and so do I",
    "A full commitment's what I'm thinking of",
    "You wouldn't get this from any other guy",
    "I just wanna tell you how I'm feeling",
    "Gotta make you understand",
    "Never gonna give you up",
    "Never gonna let you down",
    "Never gonna run around and desert you",
    "Never gonna make you cry",
    "Never gonna say goodbye",
    "Never gonna tell a lie and hurt you",
    "We've known each other for so long",
    "Your heart's been aching but you're too shy to say it",
    "Inside we both know what's been going on",
    "We know the game and we're gonna play it",
    "And if you ask me how I'm feeling",
    "Don't tell me you're too blind to see",
    "Never gonna give you up",
    "Never gonna let you down",
    "Never gonna run around and desert you",
    "Never gonna make you cry",
    "Never gonna say goodbye",
    "Never gonna tell a lie and hurt you",
    "Never gonna give you up",
    "Never gonna let you down",
    "Never gonna run around and desert you",
    "Never gonna make you cry",
    "Never gonna say goodbye",
    "Never gonna tell a lie and hurt you",
    "Never gonna give, never gonna give",
    "(Give you up)",
    "(Ooh) Never gonna give, never gonna give",
    "(Give you up)",
    "We've known each other for so long",
    "Your heart's been aching but you're too shy to say it",
    "Inside we both know what's been going on",
    "We know the game and we're gonna play it",
    "I just wanna tell you how I'm feeling",
    "Gotta make you understand",
    "Never gonna give you up",
    "Never gonna let you down",
    "Never gonna run around and desert you",
    "Never gonna make you cry",
    "Never gonna say goodbye",
    "Never gonna tell a lie and hurt you",
    "Never gonna give you up",
    "Never gonna let you down",
    "Never gonna run around and desert you",
    "Never gonna make you cry",
    "Never gonna say goodbye",
    "Never gonna tell a lie and hurt you",
    "Never gonna give you up",
    "Never gonna let you down",
    "Never gonna run around and desert you",
    "Never gonna make you cry",
};

int song_text_index = 0;

struct ArrayProcessor {
  template<bool BOUNDED, bool FIXED_LENGTH>
  void operator()( CompoundArrayMessage_<BOUNDED, FIXED_LENGTH> &array )
  {
    for ( size_t i = 0; i < array.size(); ++i ) { updateMessage( array[i] ); }
  }

  template<typename T, bool B, bool FL>
  typename std::enable_if<!std::is_same<T, std::string>::value, void>::type
  operator()( ArrayMessage_<T, B, FL> & )
  {
    // Do nothing for non-string arrays
  }

  template<typename T, bool B, bool FL>
  typename std::enable_if<std::is_same<T, std::string>::value, void>::type
  operator()( ArrayMessage_<T, B, FL> &array )
  {
    for ( size_t i = 0; i < array.size(); ++i ) {
      array.assign( i, song_text[song_text_index] );
      ++song_text_index;
      if ( song_text_index >= song_text_length )
        song_text_index = 0;
    }
  }
};

void updateMessage( Message &message )
{
  if ( message.type() == MessageTypes::String ) {
    message = song_text[song_text_index];
    ++song_text_index;
    if ( song_text_index >= song_text_length )
      song_text_index = 0;
  } else if ( message.type() == MessageTypes::Array ) {
    // Helper method from template_helpers that will call the ArrayProcessor with the actual type
    invoke_for_array_message( message.as<ArrayMessageBase>(), ArrayProcessor{} );
  } else if ( message.type() == MessageTypes::Compound ) {
    auto &compound = message.as<CompoundMessage>();
    for ( auto &child : compound.values() ) { updateMessage( *child ); }
  }
}
