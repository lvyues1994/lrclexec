#include <deque>
#include <lrclexec/ExecutionContext.h>
#include <mutex>
#include <rclcpp/guard_condition.hpp>
#include <rclcpp/waitable.hpp>
#include <stdexcept>
#include <utility>

namespace lrclexec::detail {
namespace {

struct QueueWaitable final : rclcpp::Waitable {
    explicit QueueWaitable(rclcpp::Context::SharedPtr const &context) : guard{context} {}
    ~QueueWaitable() override { clear_on_ready_callback(); }

    void post(std::function<void()> task) {
        auto const lock = std::lock_guard<std::mutex>{mutex};
        tasks.push_back(std::move(task));
        try {
            guard.trigger();
        } catch (...) {
            tasks.pop_back();
            throw;
        }
    }

    std::size_t get_number_of_ready_guard_conditions() override { return 1; }
    void add_to_wait_set(rcl_wait_set_t &waitSet) override {
        auto const lock = std::lock_guard<std::mutex>{mutex};
        guard.add_to_wait_set(waitSet);
        // A post can precede the first wait-set registration or a rebuild.
        if (not tasks.empty())
            guard.trigger();
    }
    bool is_ready(rcl_wait_set_t const &) override {
        auto const lock = std::lock_guard<std::mutex>{mutex};
        return not tasks.empty();
    }
    std::shared_ptr<void> take_data() override {
        auto batch = std::make_shared<std::deque<std::function<void()>>>();
        auto const lock = std::lock_guard<std::mutex>{mutex};
        batch->swap(tasks);
        return batch;
    }
    std::shared_ptr<void> take_data_by_entity_id(std::size_t) override { return take_data(); }
    void set_on_ready_callback(std::function<void(std::size_t, int)> callback) override {
        // One event drains the whole batch, including triggers before registration.
        guard.set_on_trigger_callback([callback = std::move(callback)](std::size_t) { callback(1, 0); });
        auto const lock = std::lock_guard<std::mutex>{mutex};
        if (not tasks.empty())
            guard.trigger(); // Re-registering after remove_node may have dropped an old event.
    }
    void clear_on_ready_callback() override { guard.set_on_trigger_callback(nullptr); }
    void execute(std::shared_ptr<void> const &data) override {
        auto const batch = std::static_pointer_cast<std::deque<std::function<void()>>>(data);
        std::exception_ptr failure;
        for (auto &task : *batch) {
            try {
                task();
            } catch (...) {
                if (not failure)
                    failure = std::current_exception();
            }
        }
        if (failure)
            std::rethrow_exception(failure);
    }

  private:
    rclcpp::GuardCondition guard;
    std::mutex mutex;
    std::deque<std::function<void()>> tasks;
};

struct ExecutionContextImpl final : ExecutionContext {
    ExecutionContextImpl(std::shared_ptr<void> owner_, NodeInterfaces interfaces_,
                         std::shared_ptr<rclcpp::Node> ordinaryNode_)
        : owner{std::move(owner_)}, interfaces{std::move(interfaces_)},
          ordinaryNode{std::move(ordinaryNode_)},
          group{interfaces.get_node_base_interface()->create_callback_group(
              rclcpp::CallbackGroupType::MutuallyExclusive)},
          queue{std::make_shared<QueueWaitable>(interfaces.get_node_base_interface()->get_context())} {
        interfaces.get_node_waitables_interface()->add_waitable(queue, group);
    }
    ~ExecutionContextImpl() override {
        interfaces.get_node_waitables_interface()->remove_waitable(queue, group);
    }
    void post(std::function<void()> task) override { queue->post(std::move(task)); }
    rclcpp::Node &node() const override {
        if (not ordinaryNode)
            throw std::logic_error{"lrclexec: node() requires rclcpp::Node; use nodeInterfaces()"};
        return *ordinaryNode;
    }
    NodeInterfaces nodeInterfaces() const override { return interfaces; }
    rclcpp::CallbackGroup::SharedPtr callbackGroup() const noexcept override { return group; }

  private:
    std::shared_ptr<void> owner;
    NodeInterfaces interfaces;
    std::shared_ptr<rclcpp::Node> ordinaryNode;
    rclcpp::CallbackGroup::SharedPtr group;
    std::shared_ptr<QueueWaitable> queue;
};

} // namespace

std::shared_ptr<ExecutionContext> makeExecutionContext(std::shared_ptr<rclcpp::Node> node) {
    if (not node) {
        throw std::invalid_argument{"lrclexec: scheduler needs a node"};
    }
    auto interfaces = NodeInterfaces{*node};
    return makeExecutionContext(node, std::move(interfaces), node);
}

std::shared_ptr<ExecutionContext> makeExecutionContext(std::shared_ptr<void> owner, NodeInterfaces interfaces,
                                                       std::shared_ptr<rclcpp::Node> ordinaryNode) {
    return std::make_shared<ExecutionContextImpl>(std::move(owner), std::move(interfaces),
                                                  std::move(ordinaryNode));
}

} // namespace lrclexec::detail
