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
    explicit ExecutionContextImpl(std::shared_ptr<rclcpp::Node> node_)
        : rosNode{std::move(node_)},
          group{rosNode->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)},
          queue{std::make_shared<QueueWaitable>(rosNode->get_node_base_interface()->get_context())} {
        rosNode->get_node_waitables_interface()->add_waitable(queue, group);
    }
    ~ExecutionContextImpl() override {
        rosNode->get_node_waitables_interface()->remove_waitable(queue, group);
    }
    void post(std::function<void()> task) override { queue->post(std::move(task)); }
    rclcpp::Node &node() const noexcept override { return *rosNode; }
    rclcpp::CallbackGroup::SharedPtr callbackGroup() const noexcept override { return group; }

  private:
    std::shared_ptr<rclcpp::Node> rosNode;
    rclcpp::CallbackGroup::SharedPtr group;
    std::shared_ptr<QueueWaitable> queue;
};

} // namespace

std::shared_ptr<ExecutionContext> makeExecutionContext(std::shared_ptr<rclcpp::Node> node) {
    if (not node) {
        throw std::invalid_argument{"lrclexec: scheduler needs a node"};
    }
    return std::make_shared<ExecutionContextImpl>(std::move(node));
}

} // namespace lrclexec::detail
