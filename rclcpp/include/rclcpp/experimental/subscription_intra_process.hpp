// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_HPP_
#define RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_HPP_

#include <rmw/rmw.h>

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rcl/error_handling.h"

#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/experimental/buffers/intra_process_buffer.hpp"
#include "rclcpp/experimental/create_intra_process_buffer.hpp"
#include "rclcpp/experimental/serialization.hpp"
#include "rclcpp/experimental/subscription_intra_process_base.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/waitable.hpp"
#include "tracetools/tracetools.h"

namespace rclcpp
{
namespace experimental
{

class SerializedContainer;

template<
  typename MessageT,
  typename Alloc = std::allocator<void>,
  typename Deleter = std::default_delete<MessageT>,
  typename CallbackMessageT = MessageT>
class SubscriptionIntraProcess : public SubscriptionIntraProcessBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SubscriptionIntraProcess)

  using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using ConstMessageSharedPtr = std::shared_ptr<const MessageT>;
  using MessageUniquePtr = std::unique_ptr<MessageT, Deleter>;
  using CallbackMessageAllocTraits = allocator::AllocRebind<CallbackMessageT, Alloc>;
  using CallbackMessageAlloc = typename CallbackMessageAllocTraits::allocator_type;
  using CallbackMessageUniquePtr = std::unique_ptr<CallbackMessageT>;
  using CallbackMessageSharedPtr = std::shared_ptr<CallbackMessageT>;

  using BufferUniquePtr = typename rclcpp::experimental::buffers::IntraProcessBuffer<
    MessageT,
    Alloc,
    Deleter
    >::UniquePtr;

  SubscriptionIntraProcess(
    AnySubscriptionCallback<CallbackMessageT, Alloc> callback,
    std::shared_ptr<Alloc> allocator,
    rclcpp::Context::SharedPtr context,
    const std::string & topic_name,
    rmw_qos_profile_t qos_profile,
    rclcpp::IntraProcessBufferType buffer_type,
    std::shared_ptr<SerializationBase> serializer)
  : SubscriptionIntraProcessBase(topic_name, qos_profile),
    any_callback_(callback), serializer_(serializer)
  {
    if (!std::is_same<MessageT, CallbackMessageT>::value &&
      !std::is_same<MessageT, rclcpp::experimental::SerializedContainer>::value &&
      !std::is_same<CallbackMessageT, rcl_serialized_message_t>::value)
    {
      throw std::runtime_error("SubscriptionIntraProcess wrong callback type");
    }

    // Create the intra-process buffer.
    buffer_ = rclcpp::experimental::create_intra_process_buffer<MessageT, Alloc, Deleter>(
      buffer_type,
      qos_profile,
      allocator);

    // Create the guard condition.
    rcl_guard_condition_options_t guard_condition_options =
      rcl_guard_condition_get_default_options();

    gc_ = rcl_get_zero_initialized_guard_condition();
    rcl_ret_t ret = rcl_guard_condition_init(
      &gc_, context->get_rcl_context().get(), guard_condition_options);

    if (RCL_RET_OK != ret) {
      throw std::runtime_error("SubscriptionIntraProcess init error initializing guard condition");
    }

    TRACEPOINT(
      rclcpp_subscription_callback_added,
      (const void *)this,
      (const void *)&any_callback_);
    // The callback object gets copied, so if registration is done too early/before this point
    // (e.g. in `AnySubscriptionCallback::set()`), its address won't match any address used later
    // in subsequent tracepoints.
#ifndef TRACETOOLS_DISABLED
    any_callback_.register_callback_for_tracing();
#endif
  }

  bool
  is_ready(rcl_wait_set_t * wait_set)
  {
    (void)wait_set;
    return buffer_->has_data();
  }

  void execute()
  {
    execute_impl<CallbackMessageT>();
  }

  void
  provide_intra_process_message(ConstMessageSharedPtr message)
  {
    buffer_->add_shared(std::move(message));
    trigger_guard_condition();
  }

  void
  provide_intra_process_message(MessageUniquePtr message)
  {
    buffer_->add_unique(std::move(message));
    trigger_guard_condition();
  }

  bool
  use_take_shared_method() const
  {
    return buffer_->use_take_shared_method();
  }

private:
  void
  trigger_guard_condition()
  {
    rcl_ret_t ret = rcl_trigger_guard_condition(&gc_);
    (void)ret;
  }

  template<typename T>
  typename std::enable_if<
    std::is_same<T, rcl_serialized_message_t>::value &&
    !std::is_same<MessageT, rclcpp::experimental::SerializedContainer>::value,
    void>::type
  execute_impl()
  {
    if (serializer_) {
      rmw_message_info_t msg_info;
      msg_info.from_intra_process = true;

      ConstMessageSharedPtr msg = buffer_->consume_shared();
      auto serialized_msg =
        serializer_->serialize_message(reinterpret_cast<const void *>(msg.get()));

      if (serialized_msg == nullptr) {
        throw std::runtime_error("Subscription intra-process could not serialize message");
      }

      if (any_callback_.use_take_shared_method()) {
        any_callback_.dispatch_intra_process(serialized_msg, msg_info);
      } else {
        throw std::runtime_error("Subscription intra-process for serialized "
                "messages does not support unique pointers.");
      }
    } else {
      throw std::runtime_error("Subscription intra-process can't handle serialized messages");
    }
  }

  template<class T>
  typename std::enable_if<
    !std::is_same<T, rcl_serialized_message_t>::value &&
    !std::is_same<MessageT, rclcpp::experimental::SerializedContainer>::value,
    void>::type
  execute_impl()
  {
    rmw_message_info_t msg_info;
    msg_info.publisher_gid = {0, {0}};
    msg_info.from_intra_process = true;

    if (any_callback_.use_take_shared_method()) {
      ConstMessageSharedPtr msg = buffer_->consume_shared();
      any_callback_.dispatch_intra_process(msg, msg_info);
    } else {
      MessageUniquePtr msg = buffer_->consume_unique();
      any_callback_.dispatch_intra_process(std::move(msg), msg_info);
    }
  }

  template<typename T>
  typename std::enable_if<
    std::is_same<T, rcl_serialized_message_t>::value &&
    std::is_same<MessageT, rclcpp::experimental::SerializedContainer>::value,
    void>::type
  execute_impl()
  {
    rmw_message_info_t msg_info {};
    msg_info.from_intra_process = true;

    if (any_callback_.use_take_shared_method()) {
      ConstMessageSharedPtr msg = buffer_->consume_shared();
      if (nullptr == msg) {
        throw std::runtime_error("Subscription intra-process could not get serialized message");
      }
      any_callback_.dispatch_intra_process(msg, msg_info);
    } else {
      throw std::runtime_error("Subscription intra-process for serialized "
              "messages does not support unique pointers.");
    }
  }

  template<class T>
  typename std::enable_if<
    !std::is_same<T, rcl_serialized_message_t>::value &&
    std::is_same<MessageT, rclcpp::experimental::SerializedContainer>::value,
    void>::type
  execute_impl()
  {
    if (serializer_) {
      ConstMessageSharedPtr serialized_container = buffer_->consume_shared();
      if (serialized_container == nullptr) {
        throw std::runtime_error("Subscription intra-process could not get serialized message");
      }

      rmw_message_info_t msg_info;
      msg_info.from_intra_process = true;

      if (any_callback_.use_take_shared_method()) {
        CallbackMessageSharedPtr msg = construct_unique();
        serializer_->deserialize_message(*serialized_container,
          reinterpret_cast<void *>(msg.get()));
        any_callback_.dispatch_intra_process(msg, msg_info);
      } else {
        CallbackMessageUniquePtr msg = construct_unique();
        serializer_->deserialize_message(*serialized_container,
          reinterpret_cast<void *>(msg.get()));
        any_callback_.dispatch_intra_process(std::move(msg), msg_info);
      }
    } else {
      throw std::runtime_error("Subscription intra-process can't handle unserialized messages");
    }
  }

  CallbackMessageUniquePtr construct_unique()
  {
    CallbackMessageUniquePtr unique_msg;
    auto ptr = CallbackMessageAllocTraits::allocate(*message_allocator_.get(), 1);
    CallbackMessageAllocTraits::construct(*message_allocator_.get(), ptr);
    unique_msg = CallbackMessageUniquePtr(ptr);

    return unique_msg;
  }

  AnySubscriptionCallback<CallbackMessageT, Alloc> any_callback_;
  BufferUniquePtr buffer_;
  std::shared_ptr<SerializationBase> serializer_;
  std::shared_ptr<CallbackMessageAlloc> message_allocator_ =
    std::make_shared<CallbackMessageAlloc>();
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_HPP_
