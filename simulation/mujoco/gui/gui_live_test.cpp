#include "Workbench.h"
#include <QApplication>
#include <QDoubleSpinBox>
#include <QListWidget>
#include <QMainWindow>
#include <QPushButton>
#include <QScreen>
#include <QTest>
#include <chrono>
#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

namespace {
void check(bool ok, char const *reason) {
    if (not ok)
        throw std::runtime_error{reason};
}
template <class Predicate> void wait(Predicate ready, char const *reason, int seconds = 30) {
    auto const deadline = std::chrono::steady_clock::now() + std::chrono::seconds{seconds};
    while (not ready() and std::chrono::steady_clock::now() < deadline)
        QTest::qWait(50);
    check(ready(), reason);
}
} // namespace
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto status = 0;
    try {
        auto app = QApplication{argc, argv};
        check(argc == 3, "provide model and screenshot directory");
        auto client = simulation::makeGuiClient(std::make_shared<rclcpp::Node>("qt_live_check"));
        auto window = simulation::makeWorkbench(*client, argv[1]);
        window->show();
        auto button = [&](char const *name) {
            auto *widget = window->findChild<QPushButton *>(name);
            check(widget, "missing button");
            return widget;
        };
        auto click = [&](char const *name) {
            auto *b = button(name);
            check(b->isEnabled(), name);
            QTest::mouseClick(b, Qt::LeftButton);
            QTest::qWait(80);
        };
        auto value = [&](char const *name, double v) {
            auto *input = window->findChild<QDoubleSpinBox *>(name);
            check(input and input->isEnabled(), name);
            input->setValue(v);
            QTest::qWait(40);
        };
        auto capture = [&](char const *name) {
            auto const path = QString::fromLocal8Bit(argv[2]) + '/' + name;
            check(window->screen()->grabWindow(window->winId()).save(path), "capture failed");
        };
        auto *points = window->findChild<QListWidget *>("waypoints");
        auto *boxes = window->findChild<QListWidget *>("obstacles");
        check(points and boxes, "missing lists");
        wait(
            [&] {
                return client->state().connected and client->state().navigation.phase == "ready" and
                       button("startTask")->isEnabled();
            },
            "GUI did not become ready");
        points->setCurrentRow(2);
        QTest::qWait(50);
        click("deleteWaypoint");
        points->setCurrentRow(0);
        QTest::qWait(50);
        value("positionX", .7);
        value("positionY", 0);
        value("headingDegrees", 0);
        click("applyEdit");
        points->setCurrentRow(1);
        QTest::qWait(50);
        value("positionX", 1);
        value("positionY", -.5);
        value("headingDegrees", -90);
        click("applyEdit");
        click("startTask");
        wait([&] { return client->state().navigation.active and not client->state().path.empty(); },
             "Qt route was not executed");
        check(not button("addWaypoint")->isEnabled(), "active route not locked");
        auto edit = [&](double x) {
            auto const before = client->state().editResponses;
            value("positionX", x);
            value("positionY", 1.5);
            click("applyEdit");
            wait([&] { return client->state().editResponses > before; }, "no scene edit confirmation", 6);
            check(client->state().editApplied and boxes->count() == 1, "scene edit not reflected in Qt");
        };
        click("addObstacle");
        edit(3);
        capture("live-running.png");
        edit(3.2);
        auto const before = client->state().editResponses;
        click("deleteObstacle");
        wait([&] { return client->state().editResponses > before; }, "no deletion confirmation", 6);
        check(client->state().editApplied and boxes->count() == 0, "deleted obstacle remains in Qt");
        wait([&] { return client->state().navigation.result == "succeeded"; },
             "Qt multipoint route did not succeed", 60);
        auto const &state = client->state();
        check(state.navigation.completed == 2, "route did not reach both points");
        auto const &q = state.scene.positions;
        check(q.size() == 9 and std::hypot(q[0] - 1, q[1] + .5) < .15, "physical arrival mismatch");
        auto const yaw = std::atan2(2 * (q[3] * q[6] + q[4] * q[5]), 1 - 2 * (q[5] * q[5] + q[6] * q[6]));
        check(std::abs(std::remainder(yaw + 1.5707963267948966, 6.283185307179586)) < .18,
              "physical heading mismatch");
        capture("live-arrived.png");
        points->setCurrentRow(0);
        QTest::qWait(50);
        value("positionX", 0);
        value("positionY", 0);
        click("applyEdit");
        click("startTask");
        wait([&] { return client->state().navigation.active != 0; }, "retry not active");
        click("stopTask");
        wait(
            [&] {
                return client->state().navigation.result == "canceled" and button("startTask")->isEnabled();
            },
            "Qt cancel did not drain");
        click("startTask");
        wait([&] { return client->state().navigation.active != 0 and not client->state().path.empty(); },
             "close check not active");
        window->close();
        window.reset();
        std::cout << "GUI_CHECK_PASSED: real route, heading, box add/move/delete, cancel and active close\n";
    } catch (std::exception const &error) {
        std::cerr << error.what() << '\n';
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}
