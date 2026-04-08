// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_BABEL_FISH_SERVICE_CLIENT_HPP
#define ROS_BABEL_FISH_BABEL_FISH_SERVICE_CLIENT_HPP

#include <rclcpp/node.hpp>
#include <ros_babel_fish/idl/type_support.hpp>
#include <ros_babel_fish/messages/compound_message.hpp>

namespace ros_babel_fish
{

class BabelFishServiceClient : public rclcpp::ClientBase
{
public:
  using SharedRequest = CompoundMessage::SharedPtr;
  using SharedResponse = CompoundMessage::SharedPtr;

  using Promise = std::promise<SharedResponse>;
  using PromiseWithRequest = std::promise<std::pair<SharedRequest, SharedResponse>>;
  using SharedPromise = std::shared_ptr<Promise>;
  using SharedPromiseWithRequest = std::shared_ptr<PromiseWithRequest>;
  using SharedFuture = std::shared_future<SharedResponse>;
  using SharedFutureWithRequest = std::shared_future<std::pair<SharedRequest, SharedResponse>>;
  using CallbackType = std::function<void( SharedFuture )>;
  using CallbackWithRequestType = std::function<void( const SharedFutureWithRequest & )>;

  struct FutureAndRequestId : rclcpp::detail::FutureAndRequestId<std::future<SharedResponse>> {
    using rclcpp::detail::FutureAndRequestId<std::future<SharedResponse>>::FutureAndRequestId;

    SharedFuture share() noexcept { return this->future.share(); }
  };

  struct SharedFutureAndRequestId : rclcpp::detail::FutureAndRequestId<SharedFuture> {
    using rclcpp::detail::FutureAndRequestId<SharedFuture>::FutureAndRequestId;
  };

  struct SharedFutureWithRequestAndRequestId
      : rclcpp::detail::FutureAndRequestId<SharedFutureWithRequest> {
    using rclcpp::detail::FutureAndRequestId<SharedFutureWithRequest>::FutureAndRequestId;
  };

  RCLCPP_SMART_PTR_DEFINITIONS( BabelFishServiceClient )

  BabelFishServiceClient( rclcpp::node_interfaces::NodeBaseInterface *node_base,
                          rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
                          const std::string &service_name,
                          ServiceTypeSupport::ConstSharedPtr type_support,
                          rcl_client_options_t client_options );

  bool take_response( CompoundMessage &response_out, rmw_request_id_t &request_header_out );

  std::shared_ptr<void> create_response() override;

  std::shared_ptr<rmw_request_id_t> create_request_header() override;

  void handle_response( const std::shared_ptr<rmw_request_id_t> &request_header,
                        const std::shared_ptr<void> &response ) override;

  FutureAndRequestId async_send_request( const SharedRequest &request );

  template<typename CallbackT, typename std::enable_if<rclcpp::function_traits::same_arguments<
                                   CallbackT, CallbackType>::value>::type * = nullptr>
  SharedFutureAndRequestId async_send_request( const SharedRequest &request, CallbackT &&cb )
  {
    Promise promise;
    auto shared_future = promise.get_future().share();
    int64_t req_id = async_send_request_impl(
        request, std::make_tuple( CallbackType{ std::forward<CallbackT>( cb ) }, shared_future,
                                  std::move( promise ) ) );
    return { std::move( shared_future ), req_id };
  }

  template<typename CallbackT, typename std::enable_if<rclcpp::function_traits::same_arguments<
                                   CallbackT, CallbackWithRequestType>::value>::type * = nullptr>
  SharedFutureWithRequestAndRequestId async_send_request( const SharedRequest &request,
                                                          CallbackT &&cb )
  {
    PromiseWithRequest promise;
    auto shared_future = promise.get_future().share();
    int64_t req_id = async_send_request_impl(
        request, std::make_tuple( CallbackWithRequestType{ std::forward<CallbackT>( cb ) }, request,
                                  shared_future, std::move( promise ) ) );
    return { std::move( shared_future ), req_id };
  }

  bool remove_pending_request( int64_t request_id );

  bool remove_pending_request( const FutureAndRequestId &future );

  bool remove_pending_request( const SharedFutureAndRequestId &future );

  bool remove_pending_request( const SharedFutureWithRequestAndRequestId &future );

  size_t prune_pending_requests();

  template<typename AllocatorT = std::allocator<int64_t>>
  size_t prune_requests_older_than( std::chrono::time_point<std::chrono::system_clock> time_point,
                                    std::vector<int64_t, AllocatorT> *pruned_requests = nullptr )
  {
    return rclcpp::detail::prune_requests_older_than_impl(
        pending_requests_, pending_requests_mutex_, time_point, pruned_requests );
  }

  void configure_introspection( const rclcpp::Clock::SharedPtr &clock,
                                const rclcpp::QoS &qos_service_event_pub,
                                rcl_service_introspection_state_t introspection_state );

protected:
  using CallbackTypeValueVariant = std::tuple<CallbackType, SharedFuture, Promise>;
  using CallbackWithRequestTypeValueVariant =
      std::tuple<CallbackWithRequestType, SharedRequest, SharedFutureWithRequest, PromiseWithRequest>;

  using CallbackInfoVariant =
      std::variant<Promise, CallbackTypeValueVariant, CallbackWithRequestTypeValueVariant>;

  int64_t async_send_request_impl( const SharedRequest &request, CallbackInfoVariant value );

  RCLCPP_DISABLE_COPY( BabelFishServiceClient )

  std::map<int64_t, std::pair<std::chrono::time_point<std::chrono::system_clock>, CallbackInfoVariant>>
      pending_requests_;
  std::mutex pending_requests_mutex_;

private:
  ServiceTypeSupport::ConstSharedPtr type_support_;
};
} // namespace ros_babel_fish

#endif // ROS_BABEL_FISH_BABEL_FISH_SERVICE_CLIENT_HPP
