// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish/detail/babel_fish_service_client.hpp"
#include "ros_babel_fish/idl/serialization.hpp"

#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

namespace ros_babel_fish
{

BabelFishServiceClient::BabelFishServiceClient(
    rclcpp::node_interfaces::NodeBaseInterface *node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    const std::string &service_name, ServiceTypeSupport::ConstSharedPtr type_support,
    rcl_client_options_t client_options )
    : ClientBase( node_base, std::move( node_graph ) ), type_support_( std::move( type_support ) )
{
  rcl_ret_t ret =
      rcl_client_init( this->get_client_handle().get(), this->get_rcl_node_handle(),
                       &type_support_->type_support_handle, service_name.c_str(), &client_options );
  if ( ret != RCL_RET_OK ) {
    if ( ret == RCL_RET_SERVICE_NAME_INVALID ) {
      auto rcl_node_handle = this->get_rcl_node_handle();
      // this will throw on any validation problem
      rcl_reset_error();
      rclcpp::expand_topic_or_service_name( service_name, rcl_node_get_name( rcl_node_handle ),
                                            rcl_node_get_namespace( rcl_node_handle ), true );
    }
    rclcpp::exceptions::throw_from_rcl_error( ret, "could not create client" );
  }
}

bool BabelFishServiceClient::take_response( CompoundMessage &response_out,
                                            rmw_request_id_t &request_header_out )
{
  std::shared_ptr<void> type_erased = create_response();
  if ( type_erased == nullptr || !take_type_erased_response( type_erased.get(), request_header_out ) )
    return false;
  response_out = CompoundMessage( type_support_->response(), std::move( type_erased ) );
  return true;
}

std::shared_ptr<void> BabelFishServiceClient::create_response()
{
  return createContainer( type_support_->response() );
}

std::shared_ptr<rmw_request_id_t> BabelFishServiceClient::create_request_header()
{
  return std::make_shared<rmw_request_id_t>();
}

void BabelFishServiceClient::handle_response( const std::shared_ptr<rmw_request_id_t> &request_header,
                                              const std::shared_ptr<void> &response )
{
  std::unique_lock<std::mutex> lock( pending_requests_mutex_ );
  const int64_t sequence_number = request_header->sequence_number;
  auto it = pending_requests_.find( sequence_number );
  if ( it == pending_requests_.end() ) {
    RCUTILS_LOG_ERROR_NAMED( "rclcpp", "Received invalid sequence number. Ignoring..." );
    return;
  }
  auto value = std::move( it->second.second );
  pending_requests_.erase( it );
  // Unlock since the callback might call this recursively
  lock.unlock();

  auto typed_response = CompoundMessage::make_shared( type_support_->response(), response );
  if ( std::holds_alternative<Promise>( value ) ) {
    auto &promise = std::get<Promise>( value );
    promise.set_value( std::move( typed_response ) );
  } else if ( std::holds_alternative<CallbackTypeValueVariant>( value ) ) {
    auto &inner = std::get<CallbackTypeValueVariant>( value );
    const auto &callback = std::get<CallbackType>( inner );
    auto &promise = std::get<Promise>( inner );
    auto &future = std::get<SharedFuture>( inner );
    promise.set_value( std::move( typed_response ) );
    callback( std::move( future ) );
  } else if ( std::holds_alternative<CallbackWithRequestTypeValueVariant>( value ) ) {
    auto &inner = std::get<CallbackWithRequestTypeValueVariant>( value );
    const auto &callback = std::get<CallbackWithRequestType>( inner );
    auto &promise = std::get<PromiseWithRequest>( inner );
    auto &future = std::get<SharedFutureWithRequest>( inner );
    auto &request = std::get<SharedRequest>( inner );
    promise.set_value( std::make_pair( std::move( request ), std::move( typed_response ) ) );
    callback( std::move( future ) );
  }
}

BabelFishServiceClient::FutureAndRequestId
BabelFishServiceClient::async_send_request( const SharedRequest &request )
{
  Promise promise;
  auto future = promise.get_future();
  auto req_id = async_send_request_impl( request, std::move( promise ) );
  return { std::move( future ), req_id };
}

int64_t BabelFishServiceClient::async_send_request_impl( const SharedRequest &request,
                                                         CallbackInfoVariant value )
{
  std::lock_guard<std::mutex> lock( pending_requests_mutex_ );
  int64_t sequence_number;
  rcl_ret_t ret = rcl_send_request( get_client_handle().get(), request->type_erased_message().get(),
                                    &sequence_number );
  if ( RCL_RET_OK != ret ) {
    rclcpp::exceptions::throw_from_rcl_error( ret, "failed to send request" );
  }

  pending_requests_.try_emplace(
      sequence_number, std::make_pair( std::chrono::system_clock::now(), std::move( value ) ) );
  return sequence_number;
}

bool BabelFishServiceClient::remove_pending_request( int64_t request_id )
{
  std::lock_guard<std::mutex> lock( pending_requests_mutex_ );
  return pending_requests_.erase( request_id ) != 0;
}

bool BabelFishServiceClient::remove_pending_request( const FutureAndRequestId &future )
{
  return remove_pending_request( future.request_id );
}

bool BabelFishServiceClient::remove_pending_request( const SharedFutureAndRequestId &future )
{
  return remove_pending_request( future.request_id );
}

bool BabelFishServiceClient::remove_pending_request( const SharedFutureWithRequestAndRequestId &future )
{
  return remove_pending_request( future.request_id );
}

size_t BabelFishServiceClient::prune_pending_requests()
{
  std::lock_guard<std::mutex> lock( pending_requests_mutex_ );
  size_t count = pending_requests_.size();
  pending_requests_.clear();
  return count;
}

void BabelFishServiceClient::configure_introspection(
    const rclcpp::Clock::SharedPtr &clock, const rclcpp::QoS &qos_service_event_pub,
    rcl_service_introspection_state_t introspection_state )
{
  rcl_publisher_options_t pub_opts = rcl_publisher_get_default_options();
  pub_opts.qos = qos_service_event_pub.get_rmw_qos_profile();

  rcl_ret_t ret = rcl_client_configure_service_introspection(
      client_handle_.get(), node_handle_.get(), clock->get_clock_handle(),
      &type_support_->type_support_handle, pub_opts, introspection_state );

  if ( RCL_RET_OK != ret ) {
    rclcpp::exceptions::throw_from_rcl_error( ret, "failed to configure client introspection" );
  }
}
} // namespace ros_babel_fish
