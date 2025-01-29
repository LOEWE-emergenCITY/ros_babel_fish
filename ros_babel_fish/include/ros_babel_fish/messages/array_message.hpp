// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_ARRAY_MESSAGE_HPP
#define ROS_BABEL_FISH_ARRAY_MESSAGE_HPP

#include "message_type_traits.hpp"
#include "ros_babel_fish/exceptions/babel_fish_exception.hpp"
#include "ros_babel_fish/messages/compound_message.hpp"

#include <rclcpp/time.hpp>
#include <rosidl_runtime_cpp/bounded_vector.hpp>

#include <memory>
#include <regex>

namespace ros_babel_fish
{

enum class ArraySize { DYNAMIC, BOUNDED, FIXED_LENGTH };

class ArrayMessageBase : public Message
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE( ArrayMessageBase )

  ArrayMessageBase( MessageMemberIntrospection member, std::shared_ptr<void> data )
      : Message( MessageTypes::Array, std::move( data ) ), member_( std::move( member ) )
  {
  }

  bool isFixedSize() const { return member_->array_size_ != 0 && !member_->is_upper_bound_; }

  bool isBounded() const { return member_->is_upper_bound_; }

  MessageType elementType() const { return MessageType( member_->type_id_ ); }

  virtual size_t size() const { return member_->size_function( data_.get() ); }

  //! @return The maximum size of the array if it is bounded or fixed size, otherwise, it will return 0.
  size_t maxSize() const { return member_->array_size_; }

  ArrayMessageBase &operator=( const ArrayMessageBase &other )
  {
    if ( this == &other )
      return *this;
    if ( elementType() != other.elementType() )
      throw BabelFishException( std::string( "Incompatible array types! (" ) +
                                std::to_string( elementType() ) + " vs " +
                                std::to_string( other.elementType() ) + ")" );
    _assign( other );
    return *this;
  }

  MessageMemberIntrospection elementIntrospection() const { return member_; }

protected:
  // Disable copy construction except for subclasses
  ArrayMessageBase( const ArrayMessageBase &other ) = default;

  void _assign( const Message &other ) override
  {
    if ( other.type() != MessageTypes::Array )
      throw BabelFishException( "Tried to assign non-array message to array message!" );
    *this = static_cast<const ArrayMessageBase &>( other );
  }

  virtual void _assign( const ArrayMessageBase &other ) = 0;

  MessageMemberIntrospection member_;
};

template<typename T, ArraySize SIZE>
class ArrayMessage_ final : public ArrayMessageBase
{
  using Reference =
      typename message_type_traits::array_type<T, SIZE == ArraySize::FIXED_LENGTH>::Reference;
  using ReturnType =
      typename message_type_traits::array_type<T, SIZE == ArraySize::FIXED_LENGTH>::ReturnType;
  using ConstReturnType =
      typename message_type_traits::array_type<T, SIZE == ArraySize::FIXED_LENGTH>::ConstReturnType;
  using ArgumentType =
      typename message_type_traits::array_type<T, SIZE == ArraySize::FIXED_LENGTH>::ArgumentType;

public:
  RCLCPP_SMART_PTR_DEFINITIONS( ArrayMessage_<T, SIZE> )

  ArrayMessage_( const ArrayMessage_ & ) = delete;

  explicit ArrayMessage_( MessageMemberIntrospection member, std::shared_ptr<void> data )
      : ArrayMessageBase( std::move( member ), std::move( data ) )
  {
  }

  ~ArrayMessage_() override = default;

  // Explicitly use Message operator to prevent hidden warnings
  using Message::operator[];

  Reference operator[]( size_t index )
  {
    if ( index >= size() )
      throw std::out_of_range( "Index was out of range of array!" );
    if constexpr ( SIZE != ArraySize::FIXED_LENGTH && std::is_same_v<T, bool> ) {
      return ( *reinterpret_cast<std::vector<T> *>( data_.get() ) )[index];
    } else {
      if ( member_->get_function == nullptr ) {
        if constexpr ( SIZE == ArraySize::DYNAMIC ) {
          return ( *reinterpret_cast<std::vector<T> *>( data_.get() ) )[index];
        }
        // This is the maximum object size divided by 1000 (so huge primitive types would fit).
        // The actual object is obviously smaller, but we have checked the index already.
        return ( *reinterpret_cast<std::array<T, 9223372036854775> *>( data_.get() ) )[index];
      }
      return *reinterpret_cast<ReturnType *>( member_->get_function( data_.get(), index ) );
    }
  }

  ConstReturnType operator[]( size_t index ) const
  {
    return const_cast<ArrayMessage_ *>( this )->operator[]( index );
  }

  Reference at( size_t index ) { return operator[]( index ); }

  ConstReturnType at( size_t index ) const { return operator[]( index ); }

  /*!
   * @param index The index at which the array element is set/overwritten
   * @param value The value with which the array element is overwritten, has to be the same as the element type.
   */
  void assign( size_t index, ArgumentType value ) { ( *this )[index] = value; }

  //! Method only for fixed length arrays to fill the array with the given value.
  template<bool ENABLED = SIZE == ArraySize::FIXED_LENGTH>
  typename std::enable_if_t<ENABLED, void> fill( ArgumentType &value )
  {
    for ( size_t i = 0; i < maxSize(); ++i ) assign( i, value );
  }

  //! Alias for assign
  void replace( size_t index, ArgumentType value ) { assign( index, value ); }

  void push_back( ArgumentType value )
  {
    static_assert( SIZE != ArraySize::FIXED_LENGTH, "Can not push back on fixed length array!" );
    if constexpr ( SIZE == ArraySize::BOUNDED ) {
      if ( member_->array_size_ <= member_->size_function( data_.get() ) ) {
        throw std::length_error( "Exceeded upper bound!" );
      }
    }
    reinterpret_cast<std::vector<T> *>( data_.get() )->push_back( value );
  }

  //! Alias for push_back
  void append( ArgumentType value ) { push_back( value ); }

  void pop_back()
  {
    if ( size() == 0 )
      return;
    static_assert( SIZE != ArraySize::FIXED_LENGTH, "Can not pop_back fixed length array!" );
    resize( size() - 1 );
  }

  void resize( size_t length )
  {
    static_assert( SIZE != ArraySize::FIXED_LENGTH, "Can not resize fixed length array!" );
    if constexpr ( SIZE == ArraySize::BOUNDED ) {
      if ( member_->array_size_ < length ) {
        throw std::length_error( "Exceeded upper bound!" );
      }
    }
    if ( member_->resize_function == nullptr )
      reinterpret_cast<std::vector<T> *>( data_.get() )->resize( length );
    else
      member_->resize_function( data_.get(), length );
  }

  size_t size() const override
  {
    if ( SIZE == ArraySize::FIXED_LENGTH )
      return member_->array_size_;

    if ( member_->size_function == nullptr )
      return reinterpret_cast<std::vector<T> *>( data_.get() )->size();
    return member_->size_function( data_.get() );
  }

  void clear()
  {
    static_assert( SIZE != ArraySize::FIXED_LENGTH, "Can not clear fixed length array!" );
    resize( 0 );
  }

  template<typename ArrayT, size_t ArrayLength, bool ENABLED = SIZE == ArraySize::FIXED_LENGTH>
  typename std::enable_if_t<ENABLED, ArrayMessage_<T, SIZE> &>
  operator=( const std::array<ArrayT, ArrayLength> &other )
  {
    if ( size() != ArrayLength )
      throw std::length_error( "Array size does not match!" );
    for ( size_t i = 0; i < ArrayLength; ++i ) assign( i, other[i] );
    return *this;
  }

private:
  void _assign( const ArrayMessageBase &other ) override
  {
    if ( other.isBounded() ) {
      _assignImpl<ArraySize::BOUNDED>( other );
      return;
    }
    if ( other.isFixedSize() ) {
      _assignImpl<ArraySize::FIXED_LENGTH>( other );
      return;
    }
    _assignImpl<ArraySize::DYNAMIC>( other );
  }

  template<ArraySize OTHER_SIZE>
  void _assignImpl( const ArrayMessageBase &other )
  {
    auto &other_typed = dynamic_cast<const ArrayMessage_<T, OTHER_SIZE> &>( other );
    if constexpr ( SIZE == ArraySize::BOUNDED ) {
      if ( other.size() > maxSize() ) {
        throw std::out_of_range(
            "Can not assign ArrayMessage as it exceeds the maximum size of this ArrayMessage!" );
      }
    }
    if constexpr ( SIZE != ArraySize::FIXED_LENGTH )
      resize( other.size() );
    for ( size_t index = 0; index < other.size(); ++index ) at( index ) = other_typed.at( index );
  }

  bool _isMessageEqual( const Message &o ) const override
  {
    const auto &other = o.as<ArrayMessageBase>();
    if ( other.isBounded() ) {
      return _isMessageEqualImpl<ArraySize::BOUNDED>( other );
    }
    if ( other.isFixedSize() ) {
      return _isMessageEqualImpl<ArraySize::FIXED_LENGTH>( other );
    }
    return _isMessageEqualImpl<ArraySize::DYNAMIC>( other );
  }

  template<ArraySize OTHER_SIZE>
  bool _isMessageEqualImpl( const ArrayMessageBase &other ) const
  {
    auto &other_typed = dynamic_cast<const ArrayMessage_<T, OTHER_SIZE> &>( other );
    if ( size() != other.size() )
      return false;
    for ( size_t index = 0; index < size(); ++index ) {
      if ( at( index ) != other_typed.at( index ) )
        return false;
    }
    return true;
  }
};

template<typename T>
using ArrayMessage = ArrayMessage_<T, ArraySize::DYNAMIC>;

template<typename T>
using FixedLengthArrayMessage = ArrayMessage_<T, ArraySize::FIXED_LENGTH>;

template<typename T>
using BoundedArrayMessage = ArrayMessage_<T, ArraySize::BOUNDED>;

//! Specialization for CompoundMessage
template<ArraySize SIZE>
class CompoundArrayMessage_ final : public ArrayMessageBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS( CompoundArrayMessage_<SIZE> )

  /*!
   * Creates a compound array, i.e., an array of compound messages in contrast to arrays of primitives such as int, bool etc.
   * If length != 0, the array is initialized with the given number of empty messages created according to the given
   * MessageTemplate.
   *
   * @param msg_template The template for the CompoundMessage
   * @param length The length of the array.
   * @param fixed_length Whether the array has fixed length or elements can be added / removed dynamically.
   * @param init Initialize the elements if length is greater than 0, if false, all elements in the container will be nullptr.
   */

  explicit CompoundArrayMessage_( MessageMemberIntrospection member, std::shared_ptr<void> data )
      : ArrayMessageBase( std::move( member ), std::move( data ) )
  {
    values_.resize( member_->size_function( data_.get() ) );
  }

  ~CompoundArrayMessage_() override { }

  std::string elementDatatype() const
  {
    MessageMembersIntrospection introspection( member_ );
    return std::string( introspection->message_namespace_ ) + "::" + introspection->message_name_;
  }

  std::string elementName() const
  {
    static const std::regex namespace_regex( "::" );
    return std::regex_replace( elementDatatype(), namespace_regex, "/" );
  }

  // Explicitly use Message operator to prevent hidden warnings
  using Message::operator[];

  CompoundMessage &operator[]( size_t index ) { return getImplementation( index ); }

  const CompoundMessage &operator[]( size_t index ) const { return getImplementation( index ); }

  CompoundMessage &at( size_t index ) { return getImplementation( index ); }

  const CompoundMessage &at( size_t index ) const { return getImplementation( index ); }

  /*!
   * @param index The index at which the array element is set/overwritten
   * @param value The value with which the array element is overwritten, has to be the same as the element type.
   */
  virtual void assign( size_t index, const CompoundMessage &value )
  {
    getImplementation( index ) = value;
  }

  //! Alias for _assign
  void replace( size_t index, const CompoundMessage &value ) { assign( index, value ); }

  void push_back( const CompoundMessage &value )
  {
    static_assert( SIZE != ArraySize::FIXED_LENGTH, "Can not push back on fixed length array!" );
    if constexpr ( SIZE == ArraySize::BOUNDED ) {
      if ( member_->array_size_ <= member_->size_function( data_.get() ) ) {
        throw std::length_error( "Exceeded upper bound!" );
      }
    }
    const size_t index = size();
    resize( index + 1 );
    assign( index, value );
  }

  //! Alias for push_back
  void append( const CompoundMessage &value ) { push_back( value ); }

  //! Creates a new CompoundMessage, appends it and returns a reference to it.
  CompoundMessage &appendEmpty()
  {
    size_t index = size();
    resize( index + 1 );
    return getImplementation( index );
  }

  void pop_back()
  {
    if ( size() == 0 )
      return;
    static_assert( SIZE != ArraySize::FIXED_LENGTH, "Can not pop_back fixed length array!" );
    resize( size() - 1 );
  }

  void resize( size_t length )
  {
    if ( length == values_.size() )
      return;
    static_assert( SIZE != ArraySize::FIXED_LENGTH, "Can not resize fixed length array!" );
    if constexpr ( SIZE == ArraySize::BOUNDED ) {
      if ( member_->array_size_ < length ) {
        throw std::length_error( "Exceeded upper bound!" );
      }
    }
    member_->resize_function( data_.get(), length );
    values_.resize( length );
    // Container may have reallocated -> content could be moved
    for ( size_t i = 0; i < values_.size(); ++i ) {
      if ( values_[i] == nullptr )
        continue;
      void *p = member_->get_function( data_.get(), i );
      if ( p == values_[i]->data_.get() )
        return; // Content was not moved
      std::shared_ptr<void> data( p, [parent = data_]( void * ) { (void)parent; } );
      values_[i]->move( data );
    }
  }

  std::vector<CompoundMessage::SharedPtr> values()
  {
    ensureInitialized();
    return values_;
  }

  std::vector<CompoundMessage::ConstSharedPtr> values() const
  {
    ensureInitialized();
    return { values_.begin(), values_.end() };
  }

  template<ArraySize OTHER_SIZE>
  CompoundArrayMessage_<SIZE> &operator=( const CompoundArrayMessage_<OTHER_SIZE> &other )
  {
    if ( this == &other )
      return *this;
    _assignImpl<OTHER_SIZE>( other );
    return *this;
  }

  void clear()
  {
    static_assert( SIZE != ArraySize::FIXED_LENGTH, "Can not clear fixed length array!" );
    member_->resize_function( data_.get(), 0 );
    values_.clear();
  }

protected:
  void onMoved() override
  {
    for ( size_t i = 0; i < values_.size(); ++i ) {
      if ( values_[i] == nullptr )
        continue;
      void *p = member_->get_function( data_.get(), i );
      if ( p == values_[i]->data_.get() )
        continue; // No need to move
      std::shared_ptr<void> data( p, [parent = data_]( void * ) { (void)parent; } );
      values_[i]->move( data );
    }
  }

private:
  void ensureInitialized() const
  {
    for ( int index = 0; index < values_.size(); ++index ) {
      if ( values_[index] != nullptr )
        continue;

      void *p = member_->get_function( data_.get(), index );
      std::shared_ptr<void> data( p, [parent = data_]( void * ) { (void)parent; } );
      values_[index] = CompoundMessage::make_shared( member_, std::move( data ) );
    }
  }

  void ensureInitialized( size_t index ) const
  {
    if ( index >= values_.size() ) {
      size_t size = member_->size_function( data_.get() );
      if ( index >= size )
        throw std::out_of_range( "Index was out of range of compound array!" );
      values_.resize( size );
    }
    if ( values_[index] == nullptr ) {
      void *p = member_->get_function( data_.get(), index );
      std::shared_ptr<void> data( p, [parent = data_]( void * ) { (void)parent; } );
      values_[index] = CompoundMessage::make_shared( member_, std::move( data ) );
    }
  }

  CompoundMessage &getImplementation( size_t index )
  {
    ensureInitialized( index );
    return *values_[index];
  }

  const CompoundMessage &getImplementation( size_t index ) const
  {
    ensureInitialized( index );
    return *values_[index];
  }

  void _assign( const ArrayMessageBase &other ) override
  {
    if ( other.isBounded() ) {
      _assignImpl<ArraySize::BOUNDED>( other );
      return;
    }
    if ( other.isFixedSize() ) {
      _assignImpl<ArraySize::FIXED_LENGTH>( other );
      return;
    }
    _assignImpl<ArraySize::DYNAMIC>( other );
  }

  template<ArraySize OTHER_SIZE>
  void _assignImpl( const ArrayMessageBase &other )
  {
    auto &other_typed = static_cast<const CompoundArrayMessage_<OTHER_SIZE> &>( other );
    if constexpr ( SIZE == ArraySize::BOUNDED ) {
      if ( other.size() > maxSize() ) {
        throw std::out_of_range(
            "Can not assign CompoundArrayMessage as it exceeds the maximum size "
            "of this CompoundArrayMessage!" );
      }
    }
    if constexpr ( SIZE != ArraySize::FIXED_LENGTH )
      resize( other.size() );
    for ( size_t index = 0; index < other.size(); ++index ) at( index ) = other_typed.at( index );
  }

  bool _isMessageEqual( const Message &o ) const override
  {
    const auto &other = o.as<ArrayMessageBase>();
    if ( other.isBounded() ) {
      return _isMessageEqualImpl<ArraySize::BOUNDED>( other );
    }
    if ( other.isFixedSize() ) {
      return _isMessageEqualImpl<ArraySize::FIXED_LENGTH>( other );
    }
    return _isMessageEqualImpl<ArraySize::DYNAMIC>( other );
  }

  template<ArraySize OTHER_SIZE>
  bool _isMessageEqualImpl( const ArrayMessageBase &other ) const
  {
    auto &other_typed = dynamic_cast<const CompoundArrayMessage_<OTHER_SIZE> &>( other );
    if ( size() != other.size() )
      return false;
    for ( size_t index = 0; index < size(); ++index ) {
      if ( at( index ) != other_typed.at( index ) )
        return false;
    }
    return true;
  }

  mutable std::vector<CompoundMessage::SharedPtr> values_;
};

using CompoundArrayMessage = CompoundArrayMessage_<ArraySize::DYNAMIC>;

using FixedLengthCompoundArrayMessage = CompoundArrayMessage_<ArraySize::FIXED_LENGTH>;

using BoundedCompoundArrayMessage = CompoundArrayMessage_<ArraySize::BOUNDED>;
} // namespace ros_babel_fish

#endif // ROS_BABEL_FISH_ARRAY_MESSAGE_HPP
