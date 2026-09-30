#pragma once

#include <functional>
#include <memory>
#include <rclcpp/node.hpp>

namespace lrclexec::detail {

struct ExecutionContext {
    virtual ~ExecutionContext() = default;
    virtual void post(std::function<void()> task) = 0;
    virtual rclcpp::Node &node() const noexcept = 0;
    virtual rclcpp::CallbackGroup::SharedPtr callbackGroup() const noexcept = 0;
};

std::shared_ptr<ExecutionContext> makeExecutionContext(std::shared_ptr<rclcpp::Node> node);

} // namespace lrclexec::detail
