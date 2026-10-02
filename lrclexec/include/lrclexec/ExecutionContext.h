#pragma once

#include <functional>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace lrclexec::detail {

using NodeInterfaces = rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeBaseInterface, rclcpp::node_interfaces::NodeClockInterface,
    rclcpp::node_interfaces::NodeLoggingInterface, rclcpp::node_interfaces::NodeParametersInterface,
    rclcpp::node_interfaces::NodeTimersInterface, rclcpp::node_interfaces::NodeTopicsInterface,
    rclcpp::node_interfaces::NodeWaitablesInterface>;

struct ExecutionContext {
    virtual ~ExecutionContext() = default;
    virtual void post(std::function<void()> task) = 0;
    virtual rclcpp::Node &node() const = 0;
    virtual NodeInterfaces nodeInterfaces() const { return NodeInterfaces{node()}; }
    virtual rclcpp::CallbackGroup::SharedPtr callbackGroup() const noexcept = 0;
};

std::shared_ptr<ExecutionContext> makeExecutionContext(std::shared_ptr<rclcpp::Node> node);
std::shared_ptr<ExecutionContext> makeExecutionContext(std::shared_ptr<void> owner, NodeInterfaces interfaces,
                                                       std::shared_ptr<rclcpp::Node> ordinaryNode);

template <class Node> std::shared_ptr<ExecutionContext> makeExecutionContext(std::shared_ptr<Node> node) {
    if (not node)
        throw std::invalid_argument{"lrclexec: scheduler needs a node"};
    auto interfaces = NodeInterfaces{*node};
    std::shared_ptr<rclcpp::Node> ordinaryNode;
    if constexpr (std::is_convertible_v<std::shared_ptr<Node>, std::shared_ptr<rclcpp::Node>>)
        ordinaryNode = node;
    return makeExecutionContext(std::move(node), std::move(interfaces), std::move(ordinaryNode));
}

} // namespace lrclexec::detail
