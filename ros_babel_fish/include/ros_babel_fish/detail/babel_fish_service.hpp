// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_BABEL_FISH_SERVICE_HPP
#define ROS_BABEL_FISH_BABEL_FISH_SERVICE_HPP

#include "ros_babel_fish/detail/any_service_callback.hpp"
#include "ros_babel_fish/idl/type_support.hpp"
#include "ros_babel_fish/messages/compound_message.hpp"
#include <rclcpp/node.hpp>

namespace ros_babel_fish
{
namespace impl
{
struct BabelFishService {
  using Request = CompoundMessage;
  using Response = CompoundMessage;
};
} // namespace impl

class BabelFishService : public rclcpp::ServiceBase,
                         public std::enable_shared_from_this<BabelFishService>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS( BabelFishService )

  //! Do not call directly, this is private API and might change. Use BabelFish::create_service.
  BabelFishService( std::shared_ptr<rcl_node_t> node, const std::string &service_name,
                    ServiceTypeSupport::ConstSharedPtr type_support, AnyServiceCallback callback,
                    rcl_service_options_t options );

  bool take_request( CompoundMessage &request_out, rmw_request_id_t &request_id_out );

  void send_response( rmw_request_id_t &request_id, CompoundMessage &response );

  std::shared_ptr<void> create_request() override;

  std::shared_ptr<rmw_request_id_t> create_request_header() override;

  void handle_request( std::shared_ptr<rmw_request_id_t> request_header,
                       std::shared_ptr<void> request ) override;

private:
  RCLCPP_DISABLE_COPY( BabelFishService )

  ServiceTypeSupport::ConstSharedPtr type_support_;
  AnyServiceCallback callback_;
};
} // namespace ros_babel_fish

#endif // ROS_BABEL_FISH_BABEL_FISH_SERVICE_HPP
