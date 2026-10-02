#include <atomic>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/experimental/executors/events_executor/simple_events_queue.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>

using namespace std::chrono_literals;
using Message = std_msgs::msg::Int64;
using BaseSubscription = rclcpp::Subscription<Message>;

// Diagnostic access to a protected SDK member; no SDK files are modified.
struct ObservedSubscription final : BaseSubscription {
    using BaseSubscription::BaseSubscription;
    bool registered() {
        auto const lock = std::lock_guard<std::recursive_mutex>{callback_mutex_};
        return static_cast<bool>(on_new_message_callback_);
    }
};
struct Observation {
    std::atomic<std::uint64_t> calls{0};
    std::atomic<std::int64_t> last{0};
};
struct TraceQueue final : rclcpp::experimental::executors::SimpleEventsQueue {
    using Event = rclcpp::experimental::executors::ExecutorEvent;
    using Type = rclcpp::experimental::executors::ExecutorEventType;
    void enqueue(Event const &event) override {
        if (event.type == Type::SUBSCRIPTION_EVENT) {
            auto const lock = std::lock_guard<std::mutex>{mutex};
            enqueued[event.entity_key] += event.num_events;
        }
        SimpleEventsQueue::enqueue(event);
    }
    bool dequeue(Event &event, std::chrono::nanoseconds timeout = std::chrono::nanoseconds::max()) override {
        auto const taken = SimpleEventsQueue::dequeue(event, timeout);
        if (taken and event.type == Type::SUBSCRIPTION_EVENT) {
            auto const lock = std::lock_guard<std::mutex>{mutex};
            dequeued[event.entity_key] += event.num_events;
        }
        return taken;
    }
    void report(void const *const key) {
        auto const lock = std::lock_guard<std::mutex>{mutex};
        std::cout << "subscription_events_enqueued=" << enqueued[key]
                  << " subscription_events_dequeued=" << dequeued[key] << '\n';
    }
    std::mutex mutex;
    std::unordered_map<void const *, std::uint64_t> enqueued, dequeued;
};
void require(bool const condition, char const *const message) {
    if (not condition)
        throw std::runtime_error{message};
}
template <class Function> bool until(Function condition, std::chrono::milliseconds timeout = 3s) {
    auto const deadline = std::chrono::steady_clock::now() + timeout;
    while (not condition()) {
        if (std::chrono::steady_clock::now() >= deadline)
            return false;
        std::this_thread::sleep_for(1ms);
    }
    return true;
}
struct Dispatch {
    template <class Function> auto run(Function function) {
        using Result = decltype(function());
        auto promise = std::make_shared<std::promise<Result>>();
        auto future = promise->get_future();
        {
            auto const lock = std::lock_guard<std::mutex>{mutex};
            require(not work, "dispatch already occupied");
            work = [promise, function = std::move(function)]() mutable {
                try {
                    if constexpr (std::is_void_v<Result>) {
                        function();
                        promise->set_value();
                    } else
                        promise->set_value(function());
                } catch (...) {
                    promise->set_exception(std::current_exception());
                }
            };
        }
        if (future.wait_for(5s) != std::future_status::ready) {
            std::cerr << "dispatch timeout; preserving callback borrows\n";
            std::_Exit(124);
        }
        return future.get();
    }
    void tick() {
        std::function<void()> task;
        {
            auto const lock = std::lock_guard<std::mutex>{mutex};
            task = std::move(work);
            work = {};
        }
        if (task)
            task();
    }
    std::mutex mutex;
    std::function<void()> work;
};
struct Spin {
    explicit Spin(rclcpp::Executor &executor_)
        : executor{executor_}, worker{[this] {
              try {
                  executor.spin();
              } catch (std::exception const &error) {
                  std::cerr << "executor exception: " << error.what() << '\n';
                  std::_Exit(125);
              }
          }} {}
    ~Spin() {
        executor.cancel();
        worker.join();
    }
    rclcpp::Executor &executor;
    std::thread worker;
};
struct Publishing {
    explicit Publishing(rclcpp::Publisher<Message>::SharedPtr publisher_)
        : publisher{std::move(publisher_)}, worker{[this] {
              while (not stopped.load()) {
                  if (enabled.load())
                      send();
                  std::this_thread::sleep_for(1ms);
              }
          }} {}
    ~Publishing() {
        stopped = true;
        worker.join();
    }
    void send() {
        auto message = Message{};
        message.data = sequence.fetch_add(1) + 1;
        publisher->publish(message);
        ++published;
    }
    rclcpp::Publisher<Message>::SharedPtr publisher;
    std::atomic<bool> enabled{false}, stopped{false};
    std::atomic<std::int64_t> sequence{0};
    std::atomic<std::uint64_t> published{0};
    std::thread worker;
};
int experiment(std::string const &mode) {
    auto node = std::make_shared<rclcpp::Node>("native_topic_reuse_target");
    auto peer = std::make_shared<rclcpp::Node>("native_topic_reuse_peer");
    auto const topic = std::string{"native_topic_reuse"};
    auto const qos = rclcpp::QoS{32}.reliable().durability_volatile();
    auto control = std::make_shared<Observation>();
    auto dynamic = std::make_shared<Observation>();
    auto permanent = peer->create_subscription<Message>(topic, qos, [control](Message const &message) {
        control->last = message.data;
        ++control->calls;
    });
    auto publisher = peer->create_publisher<Message>(topic, qos);
    auto dispatch = Dispatch{};
    auto pulse = node->create_wall_timer(1ms, [&dispatch] { dispatch.tick(); });
    auto create = [&] {
        auto callback = [dynamic](Message const &message) {
            dynamic->last = message.data;
            ++dynamic->calls;
        };
        auto options = rclcpp::SubscriptionOptions{};
        auto typedCallback = rclcpp::AnySubscriptionCallback<Message>{};
        typedCallback.set(std::move(callback));
        auto result = std::make_shared<ObservedSubscription>(
            node->get_node_base_interface().get(), rclcpp::get_message_type_support_handle<Message>(), topic,
            qos, typedCallback, options, BaseSubscription::MessageMemoryStrategyType::create_default());
        result->post_init_setup(node->get_node_base_interface().get(), qos, options);
        node->get_node_topics_interface()->add_subscription(result, options.callback_group);
        return result;
    };
    auto subscription = std::shared_ptr<ObservedSubscription>{};
    std::unique_ptr<rclcpp::Executor> executor;
    TraceQueue *trace = nullptr;
    if (mode == "events") {
        auto queue = std::make_unique<TraceQueue>();
        trace = queue.get();
        executor = std::make_unique<rclcpp::experimental::executors::EventsExecutor>(std::move(queue));
    } else if (mode == "multi")
        executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions{}, 4);
    else if (mode == "single")
        executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    else
        throw std::invalid_argument{"mode must be single, multi or events"};
    executor->add_node(node);
    executor->add_node(peer);
    auto spin = Spin{*executor};
    auto publishing = Publishing{publisher};
    subscription = dispatch.run(create);
    auto const initialMatched = until([&] {
        return publisher->get_subscription_count() == 2 and subscription->get_publisher_count() == 1;
    });
    if (not initialMatched)
        std::cerr << "initial matched_subscriptions=" << publisher->get_subscription_count()
                  << " matched_publishers=" << subscription->get_publisher_count() << '\n';
    require(initialMatched, "initial DDS matching failed");
    publishing.send();
    require(until([&] { return dynamic->calls.load() == 1 and control->calls.load() == 1; }),
            "initial callback did not execute");
    void const *oldKey = subscription->get_subscription_handle().get();
    auto oldWeak = std::weak_ptr<ObservedSubscription>{subscription};
    auto const initiallyRegistered = dispatch.run([&] { return subscription->registered(); });
    auto attempts = 0;
    auto reused = false;
    auto rounds = 0;
    for (; rounds < (mode == "events" ? 32 : 1); ++rounds) {
        oldKey = subscription->get_subscription_handle().get();
        oldWeak = subscription;
        dynamic->calls = 0;
        auto const controlBefore = control->calls.load();
        publishing.send();
        require(until([&] { return dynamic->calls.load() == 1 and control->calls.load() > controlBefore; }),
                "round baseline callback did not execute");
        attempts = 0;
        reused = dispatch.run([&] {
            subscription.reset();
            if (not oldWeak.expired()) {
                subscription = create();
                ++attempts;
                return false;
            }
            for (; attempts < 512; ++attempts) {
                subscription = create();
                if (subscription->get_subscription_handle().get() == oldKey) {
                    ++attempts;
                    return true;
                }
                subscription.reset();
            }
            subscription = create();
            return false;
        });
        require(until([&] {
                    return subscription->get_publisher_count() == 1 and
                           publisher->get_subscription_count() == 2;
                }),
                "replacement DDS matching failed");
        if (reused)
            break;
    }
    std::this_thread::sleep_for(30ms);
    auto const replacementRegistered = dispatch.run([&] { return subscription->registered(); });
    std::cout << "mode=" << mode << " handle_reused=" << reused << " rounds=" << rounds + (reused ? 1 : 0)
              << " attempts=" << attempts << " old_key=" << oldKey
              << " new_key=" << subscription->get_subscription_handle().get()
              << " old_weak_expired=" << oldWeak.expired() << " initial_notifier=" << initiallyRegistered
              << " replacement_notifier=" << replacementRegistered
              << " matched_publishers=" << subscription->get_publisher_count()
              << " matched_subscriptions=" << publisher->get_subscription_count() << '\n';
    auto const before = publishing.published.load();
    publishing.enabled = true;
    std::this_thread::sleep_for(250ms);
    publishing.enabled = false;
    std::this_thread::sleep_for(20ms);
    auto const afterCalls = dynamic->calls.load();
    auto const taken = dispatch.run([&] {
        auto message = Message{};
        auto info = rclcpp::MessageInfo{};
        auto const result = subscription->take(message, info);
        std::cout << "manual_take=" << result << " taken_sequence=" << message.data << '\n';
        return result;
    });
    std::cout << "published_delta=" << publishing.published.load() - before
              << " permanent_callbacks=" << control->calls.load()
              << " permanent_last=" << control->last.load() << " replacement_callbacks=" << afterCalls - 1
              << " replacement_last=" << dynamic->last.load() << '\n';
    if (trace)
        trace->report(oldKey);
    auto const missing = afterCalls == 1 and taken and replacementRegistered == false;
    if (mode == "events" and missing) {
        auto const registeredAfterRefresh = dispatch.run([&] {
            executor->remove_node(node);
            executor->add_node(node);
            return subscription->registered();
        });
        publishing.enabled = true;
        auto const recovered = until([&] { return dynamic->calls.load() > afterCalls; });
        publishing.enabled = false;
        std::cout << "explicit_detach_readd_notifier=" << registeredAfterRefresh
                  << " callback_recovered=" << recovered << '\n';
        std::cout << "SDK_SUBSCRIPTION_CACHE_FAILURE_REPRODUCED\n" << std::flush;
        return 2;
    }
    require(afterCalls > 1, "replacement did not receive messages");
    std::cout << "replacement delivered normally\n" << std::flush;
    return mode == "events" and not reused ? 3 : 0;
}
int main(int argc, char **argv) {
    try {
        rclcpp::init(0, nullptr, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
        auto const result = experiment(argc > 1 ? argv[1] : "events");
        rclcpp::shutdown();
        return result;
    } catch (std::exception const &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
