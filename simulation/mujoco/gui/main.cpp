#include "Workbench.h"
#include <QApplication>
#include <QMainWindow>
#include <QTimer>
#include <iostream>
#include <lrclexec/SignalStop.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    auto stop = lexec::inplace_stop_source{};
    auto signals = lrclexec::SignalStop{stop};
    auto options = rclcpp::InitOptions{};
    options.shutdown_on_signal = false;
    rclcpp::init(argc, argv, options, rclcpp::SignalHandlerOptions::None);
    auto status = 0;
    try {
        auto app = QApplication{argc, argv};
        auto node = std::make_shared<rclcpp::Node>("mujoco_qt_workbench");
        auto client = simulation::makeGuiClient(node);
        auto window = simulation::makeWorkbench(*client, node->declare_parameter<std::string>("model"));
        auto timer = QTimer{};
        QObject::connect(&timer, &QTimer::timeout, &app, [&] {
            if (stop.stop_requested())
                app.quit();
        });
        timer.start(30);
        window->show();
        status = app.exec();
    } catch (std::exception const &error) {
        std::cerr << error.what() << '\n';
        status = 1;
    }
    rclcpp::shutdown();
    return status or signals.error() ? 1 : 0;
}
