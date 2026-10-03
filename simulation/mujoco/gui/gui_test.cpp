#include "SceneView.h"
#include "Workbench.h"
#include <QApplication>
#include <QDoubleSpinBox>
#include <QListWidget>
#include <QMainWindow>
#include <QPushButton>
#include <QScreen>
#include <QTest>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool ok, char const *reason) {
    if (not ok)
        throw std::runtime_error{reason};
}
struct Client final : simulation::GuiClient {
    explicit Client(std::string const &file) : physics{simulation::makePhysics(file)} {
        for (int i = 0; i < 250; ++i)
            physics->step({});
        snapshot.connected = true;
        snapshot.navigation.phase = "ready";
        snapshot.scene = physics->scene();
    }
    void poll() override { snapshot.scene = physics->scene(); }
    simulation::GuiState const &state() const override { return snapshot; }
    bool sendRoute(std::vector<simulation::Waypoint> const &route) override {
        snapshot.activeRoute = route;
        ++snapshot.routeRevision;
        snapshot.navigation.active = ++snapshot.navigation.accepted;
        snapshot.navigation.phase = "running";
        snapshot.navigation.count = route.size();
        snapshot.navigation.index = 1;
        return true;
    }
    void cancel() override {
        snapshot.navigation.active = 0;
        snapshot.navigation.phase = "ready";
        snapshot.navigation.result = "canceled";
    }
    bool edit(simulation::ObstacleEdit const &edit) override {
        if (delayEdit) {
            pendingEdit = edit;
            snapshot.editPending = true;
            return true;
        }
        applyEdit(edit);
        return true;
    }
    void applyEdit(simulation::ObstacleEdit const &edit) {
        auto const result = physics->editObstacle(edit);
        snapshot.editPending = false;
        snapshot.editApplied = result.applied;
        ++snapshot.editResponses;
        snapshot.feedback = result.applied ? "障碍物已应用" : "障碍物未应用：" + result.reason;
    }
    bool delayEdit = false;
    std::optional<simulation::ObstacleEdit> pendingEdit;
    std::unique_ptr<simulation::Physics> physics;
    simulation::GuiState snapshot;
};
} // namespace
int main(int argc, char **argv) {
    auto app = QApplication{argc, argv};
    try {
        if (argc != 3)
            throw std::invalid_argument{"provide model and screenshot directory"};
        auto client = Client{argv[1]};
        auto window = simulation::makeWorkbench(client, argv[1]);
        window->show();
        QTest::qWait(400);
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
        auto *points = window->findChild<QListWidget *>("waypoints");
        auto *boxes = window->findChild<QListWidget *>("obstacles");
        check(points and boxes, "missing lists");
        auto capture = [&](char const *name) {
            auto const path = QString::fromLocal8Bit(argv[2]) + '/' + name;
            check(window->screen()->grabWindow(window->winId()).save(path), "window capture failed");
        };
        capture("ready.png");
        click("addWaypoint");
        value("positionX", 1.2);
        value("positionY", 1.5);
        value("headingDegrees", 90);
        click("applyEdit");
        check(points->count() == 4, "GUI did not add waypoint");
        click("moveWaypointUp");
        check(points->currentRow() == 2, "GUI reorder failed");
        client.delayEdit = true;
        click("addObstacle");
        value("positionX", 1);
        value("positionY", 1);
        click("applyEdit");
        check(not points->isEnabled() and not boxes->isEnabled() and not button("selectTool")->isEnabled() and
                  not button("addWaypoint")->isEnabled(),
              "pending edit allowed selection changes");
        check(client.pendingEdit.has_value(), "missing delayed edit");
        client.applyEdit(*client.pendingEdit);
        client.delayEdit = false;
        QTest::qWait(80);
        check(boxes->count() == 1 and client.snapshot.scene.revision == 1,
              "GUI did not apply physics obstacle");
        check(not button("deleteWaypoint")->isEnabled(), "obstacle selection left waypoint delete enabled");
        points->setCurrentRow(2);
        QTest::qWait(60);
        boxes->setCurrentRow(0);
        QTest::qWait(60);
        points->setCurrentRow(2);
        QTest::qWait(60);
        check(button("deleteWaypoint")->isEnabled(), "same-item re-selection failed");
        click("startTask");
        check(client.snapshot.activeRoute.size() == 4 and not button("addWaypoint")->isEnabled(),
              "running route not locked");
        click("addObstacle");
        value("positionX", 1);
        value("positionY", 1);
        click("applyEdit");
        check(not client.snapshot.editApplied and boxes->count() == 1, "overlap accepted in GUI");
        capture("rejected.png");
        value("positionX", 3);
        value("positionY", 1);
        click("applyEdit");
        check(boxes->count() == 2 and client.snapshot.editApplied, "obstacle edit during navigation failed");
        capture("running.png");
        click("deleteObstacle");
        check(boxes->count() == 1, "GUI obstacle deletion failed");
        click("stopTask");
        check(button("addWaypoint")->isEnabled(), "stopped route remained locked");
        click("addWaypoint");
        simulation::SceneView *view = nullptr;
        for (auto *candidate : QGuiApplication::allWindows())
            if (auto *scene = dynamic_cast<simulation::SceneView *>(candidate))
                view = scene;
        check(view, "missing native MuJoCo view");
        QTest::mouseClick(view, Qt::LeftButton, Qt::NoModifier,
                          QPoint{view->width() / 2, view->height() / 2});
        QTest::qWait(100);
        auto *pickedX = window->findChild<QDoubleSpinBox *>("positionX");
        check(std::abs(pickedX->value() - 2) < .1, "floor picking is not in scene coordinates");
        click("discardEdit");
        window->resize(960, 660);
        QTest::qWait(200);
        capture("compact.png");
        window->close();
        window.reset();
        std::cout << "Qt waypoint editing, runtime obstacles, overlap rejection, selection, picking and "
                     "close passed\n";
    } catch (std::exception const &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
