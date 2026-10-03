#include "Workbench.h"
#include "SceneView.h"
#include <QButtonGroup>
#include <QCloseEvent>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QMainWindow>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSplitter>
#include <QStatusBar>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

namespace simulation {
namespace {
constexpr double pi = 3.14159265358979323846;
enum struct Selection { none, point, box };
struct Workbench final : QMainWindow {
    explicit Workbench(GuiClient &client_, std::string const &model) : client{client_} {
        setWindowTitle(QStringLiteral("lrclexec · Qt / MuJoCo 导航仿真"));
        setObjectName("navigationWorkbench");
        resize(1240, 820);
        setMinimumSize(960, 660);
        auto *central = new QWidget(this);
        auto *layout = new QVBoxLayout(central);
        layout->setContentsMargins(12, 12, 12, 8);
        setCentralWidget(central);
        buildToolbar(layout);
        auto *splitter = new QSplitter(Qt::Horizontal, central);
        layout->addWidget(splitter, 1);
        buildList(splitter);
        buildScene(splitter, model);
        buildInspector(splitter);
        splitter->setSizes({210, 760, 240});
        splitter->setStretchFactor(0, 0);
        splitter->setStretchFactor(1, 1);
        splitter->setStretchFactor(2, 0);
        splitter->setChildrenCollapsible(false);
        statusLabel = new QLabel(QStringLiteral("正在连接仿真与导航…"), this);
        statusLabel->setObjectName("navigationStatus");
        statusBar()->addWidget(statusLabel, 1);
        connection = new QLabel(this);
        statusBar()->addPermanentWidget(connection);
        setStyleSheet(QStringLiteral(
            "QMainWindow {background:#f2f3f5;} QWidget {font-size:14px;}"
            "QPushButton {padding:7px 10px;} QListWidget {background:#fafbfc; border:1px solid #d8dce2;}"
            "QListWidget::item {padding:11px 6px;} QDoubleSpinBox {padding:6px;}"
            "QPushButton#startTask, QPushButton#applyEdit {background:#2867c7;color:white;border:1px solid "
            "#2867c7;border-radius:4px;}"
            "QPushButton#startTask:disabled, QPushButton#applyEdit:disabled "
            "{background:#c9d1de;color:#667080;}"
            "QLabel#operationFeedback {color:#42556e;}"));
        refreshPoints();
        pointList->setCurrentRow(0);
        auto *timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, [this] { refresh(); });
        timer->start(30);
        refresh();
    }
    QPushButton *button(QString const &text, char const *name, QBoxLayout *layout) {
        auto *result = new QPushButton(text, this);
        result->setObjectName(name);
        layout->addWidget(result);
        return result;
    }
    void buildToolbar(QVBoxLayout *layout) {
        auto *row = new QHBoxLayout;
        layout->addLayout(row);
        selectTool = button(QStringLiteral("选择"), "selectTool", row);
        addPoint = button(QStringLiteral("＋ 任务点"), "addWaypoint", row);
        addBox = button(QStringLiteral("＋ 障碍物"), "addObstacle", row);
        row->addStretch();
        start = button(QStringLiteral("开始任务"), "startTask", row);
        stop = button(QStringLiteral("停止任务"), "stopTask", row);
        connect(selectTool, &QPushButton::clicked, this, [this] { discard(); });
        connect(addPoint, &QPushButton::clicked, this, [this] { beginPoint(); });
        connect(addBox, &QPushButton::clicked, this, [this] { beginBox(); });
        connect(start, &QPushButton::clicked, this, [this] {
            if (client.sendRoute(points))
                feedback->setText(QStringLiteral("正在提交任务…"));
            refresh();
        });
        connect(stop, &QPushButton::clicked, this, [this] {
            client.cancel();
            refresh();
        });
    }
    void buildList(QSplitter *splitter) {
        auto *panel = new QWidget(splitter);
        auto *col = new QVBoxLayout(panel);
        col->setContentsMargins(0, 0, 8, 0);
        col->addWidget(new QLabel(QStringLiteral("导航任务 · 按序逐点到达"), panel));
        pointList = new QListWidget(panel);
        pointList->setObjectName("waypoints");
        col->addWidget(pointList, 1);
        auto *row = new QHBoxLayout;
        col->addLayout(row);
        up = button(QStringLiteral("上移"), "moveWaypointUp", row);
        down = button(QStringLiteral("下移"), "moveWaypointDown", row);
        removePoint = button(QStringLiteral("删除"), "deleteWaypoint", row);
        connect(up, &QPushButton::clicked, this, [this] { reorder(-1); });
        connect(down, &QPushButton::clicked, this, [this] { reorder(1); });
        connect(removePoint, &QPushButton::clicked, this, [this] {
            if (busy() or pointList->currentRow() < 0)
                return;
            auto const index = pointList->currentRow();
            points.erase(points.begin() + index);
            draftEdited = true;
            refreshPoints();
            pointList->setCurrentRow(std::min(index, static_cast<int>(points.size()) - 1));
            choosePoint(pointList->currentRow());
        });
        auto *note = new QLabel(QStringLiteral("执行时锁定任务点；停止后可编辑。"), panel);
        note->setWordWrap(true);
        col->addWidget(note);
        boxLabel = new QLabel(QStringLiteral("场景障碍物 · 0 / 16"), panel);
        col->addWidget(boxLabel);
        boxList = new QListWidget(panel);
        boxList->setObjectName("obstacles");
        boxList->setMaximumHeight(210);
        col->addWidget(boxList);
        auto *fixed = new QLabel(QStringLiteral("墙体和原有箱体不可编辑"), panel);
        fixed->setWordWrap(true);
        col->addWidget(fixed);
        connect(pointList, &QListWidget::currentRowChanged, this, [this](int index) { choosePoint(index); });
        connect(boxList, &QListWidget::currentRowChanged, this, [this](int index) {
            if (index >= 0)
                chooseBox(static_cast<std::size_t>(boxList->item(index)->data(Qt::UserRole).toUInt()));
        });
    }
    void buildScene(QSplitter *splitter, std::string const &file) {
        auto *panel = new QWidget(splitter);
        auto *col = new QVBoxLayout(panel);
        col->setContentsMargins(0, 0, 0, 0);
        auto *row = new QHBoxLayout;
        col->addLayout(row);
        row->addWidget(new QLabel(QStringLiteral("MuJoCo 场景"), panel));
        row->addStretch();
        top = button(QStringLiteral("俯视编辑"), "topView", row);
        orbit = button(QStringLiteral("3D 观察"), "orbitView", row);
        top->setCheckable(true);
        orbit->setCheckable(true);
        orbit->setChecked(true);
        auto *group = new QButtonGroup(this);
        group->addButton(top);
        group->addButton(orbit);
        view = new SceneView(file, [this](ScenePick const &pick) { picked(pick); });
        auto *container = QWidget::createWindowContainer(view, panel);
        container->setObjectName("mujocoViewport");
        container->setMinimumSize(360, 380);
        col->addWidget(container, 1);
        auto *legend = new QLabel(QStringLiteral("蓝色虚线：任务顺序　绿色实线：Nav2 实际路径"), panel);
        legend->setWordWrap(true);
        col->addWidget(legend);
        auto *hint =
            new QLabel(QStringLiteral("俯视：点击定位与选择　3D：左拖旋转、右拖平移、滚轮缩放"), panel);
        hint->setWordWrap(true);
        col->addWidget(hint);
        connect(top, &QPushButton::clicked, this, [this] { view->topView(true); });
        connect(orbit, &QPushButton::clicked, this, [this] { view->topView(false); });
    }
    QDoubleSpinBox *coordinate(QFormLayout *form, QString const &label, char const *name, double low,
                               double high) {
        auto *field = new QDoubleSpinBox(this);
        field->setObjectName(name);
        field->setRange(low, high);
        field->setDecimals(2);
        field->setSingleStep(.1);
        form->addRow(label, field);
        connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this] { dirty = true; });
        return field;
    }
    void buildInspector(QSplitter *splitter) {
        auto *panel = new QWidget(splitter);
        auto *col = new QVBoxLayout(panel);
        col->setContentsMargins(8, 0, 0, 0);
        title = new QLabel(QStringLiteral("对象属性"), panel);
        col->addWidget(title);
        properties = new QWidget(panel);
        auto *form = new QFormLayout(properties);
        form->setContentsMargins(0, 12, 0, 12);
        x = coordinate(form, "X / m", "positionX", -2, 6);
        y = coordinate(form, "Y / m", "positionY", -3, 3);
        yaw = coordinate(form, QStringLiteral("朝向 / °"), "headingDegrees", -180, 180);
        yaw->setSingleStep(5);
        yawLabel = form->labelForField(yaw);
        col->addWidget(properties);
        sizeLabel = new QLabel(QStringLiteral("固定尺寸：0.8 × 0.6 × 0.6 m\n首版为轴对齐箱体"), panel);
        sizeLabel->setWordWrap(true);
        col->addWidget(sizeLabel);
        apply = button(QStringLiteral("应用修改"), "applyEdit", col);
        discardButton = button(QStringLiteral("放弃预览"), "discardEdit", col);
        removeBox = button(QStringLiteral("删除障碍物"), "deleteObstacle", col);
        feedback = new QLabel(panel);
        feedback->setObjectName("operationFeedback");
        feedback->setWordWrap(true);
        col->addWidget(feedback);
        retry = button(QStringLiteral("重试原任务剩余点"), "retryRemaining", col);
        retry->hide();
        col->addStretch();
        auto *note =
            new QLabel(QStringLiteral("障碍物应用成功后才参与碰撞和激光；预览不会改变仿真。"), panel);
        note->setWordWrap(true);
        col->addWidget(note);
        connect(apply, &QPushButton::clicked, this, [this] { applySelection(); });
        connect(discardButton, &QPushButton::clicked, this, [this] { discard(); });
        connect(removeBox, &QPushButton::clicked, this, [this] {
            if (selection == Selection::box and not isNew and client.edit({boxSlot, false}))
                pendingEditSlot = boxSlot;
        });
        connect(retry, &QPushButton::clicked, this, [this] {
            auto const &s = client.state();
            if (s.navigation.completed < s.activeRoute.size()) {
                auto remaining = std::vector<Waypoint>{
                    s.activeRoute.begin() + static_cast<std::ptrdiff_t>(s.navigation.completed),
                    s.activeRoute.end()};
                client.sendRoute(remaining);
            }
        });
    }
    bool busy() const {
        auto const &s = client.state();
        return s.routePending or s.cancelPending or s.navigation.active != 0 or s.navigation.pending != 0 or
               s.navigation.phase == "stopping";
    }
    void coordinates(Waypoint const &point) {
        auto bx = QSignalBlocker{x}, by = QSignalBlocker{y}, bh = QSignalBlocker{yaw};
        x->setValue(point.x);
        y->setValue(point.y);
        yaw->setValue(point.yaw * 180 / pi);
        dirty = false;
    }
    void choosePoint(int index) {
        if (not x or index < 0 or static_cast<std::size_t>(index) >= points.size()) {
            selection = Selection::none;
            if (properties)
                showInspector();
            return;
        }
        selection = Selection::point;
        pointIndex = index;
        isNew = false;
        coordinates(points[static_cast<std::size_t>(index)]);
        auto blocker = QSignalBlocker{boxList};
        boxList->setCurrentRow(-1);
        showInspector();
    }
    void chooseBox(std::size_t slot) {
        auto const &positions = client.state().scene.mocapPositions;
        if (slot >= obstacleCapacity or positions.size() != 3 * obstacleCapacity or
            positions[3 * slot + 2] < 0)
            return;
        selection = Selection::box;
        boxSlot = slot;
        isNew = false;
        coordinates({positions[3 * slot], positions[3 * slot + 1], 0});
        auto blocker = QSignalBlocker{pointList};
        pointList->setCurrentRow(-1);
        showInspector();
    }
    void beginPoint() {
        if (busy() or points.size() >= 64)
            return;
        selection = Selection::point;
        isNew = true;
        pointIndex = -1;
        coordinates({3, 1, 0});
        dirty = true;
        setTop();
        showInspector();
    }
    void beginBox() {
        auto const &positions = client.state().scene.mocapPositions;
        if (positions.size() != 3 * obstacleCapacity or client.state().editPending)
            return;
        for (std::size_t slot = 0; slot < obstacleCapacity; ++slot) {
            if (positions[3 * slot + 2] >= 0)
                continue;
            selection = Selection::box;
            boxSlot = slot;
            isNew = true;
            coordinates({3, 1, 0});
            dirty = true;
            setTop();
            showInspector();
            return;
        }
        feedback->setText(QStringLiteral("已达到 16 个障碍物的上限"));
    }
    void setTop() {
        top->setChecked(true);
        view->topView(true);
    }
    void discard() {
        isNew = false;
        dirty = false;
        if (selection == Selection::point and pointIndex >= 0)
            choosePoint(pointIndex);
        else if (selection == Selection::box) {
            selection = Selection::none;
            chooseBox(boxSlot);
        } else
            selection = Selection::none;
        showInspector();
    }
    void picked(ScenePick const &pick) {
        if (client.state().editPending)
            return;
        if (not isNew and not dirty) {
            if (pick.waypoint) {
                pointList->setCurrentRow(static_cast<int>(*pick.waypoint));
                choosePoint(static_cast<int>(*pick.waypoint));
                return;
            }
            if (pick.obstacle) {
                chooseBox(*pick.obstacle);
                return;
            }
        }
        if (not top->isChecked() or selection == Selection::none or
            (selection == Selection::point and busy()) or
            (selection == Selection::box and client.state().editPending))
            return;
        x->setValue(pick.x);
        y->setValue(pick.y);
        dirty = true;
    }
    void applySelection() {
        if (selection == Selection::point and not busy()) {
            auto const point = Waypoint{x->value(), y->value(), yaw->value() * pi / 180};
            if (isNew) {
                points.push_back(point);
                pointIndex = static_cast<int>(points.size()) - 1;
            } else if (pointIndex >= 0)
                points[static_cast<std::size_t>(pointIndex)] = point;
            isNew = false;
            dirty = false;
            draftEdited = true;
            refreshPoints();
            pointList->setCurrentRow(pointIndex);
            choosePoint(pointIndex);
            feedback->setText(QStringLiteral("任务点草稿已更新"));
        } else if (selection == Selection::box) {
            if (client.edit({boxSlot, true, x->value(), y->value()})) {
                pendingEditSlot = boxSlot;
                feedback->setText(QStringLiteral("正在应用障碍物…"));
            }
        }
        showInspector();
    }
    void reorder(int delta) {
        auto const index = pointList->currentRow(), next = index + delta;
        if (busy() or index < 0 or next < 0 or next >= static_cast<int>(points.size()))
            return;
        std::swap(points[static_cast<std::size_t>(index)], points[static_cast<std::size_t>(next)]);
        draftEdited = true;
        refreshPoints();
        pointList->setCurrentRow(next);
        choosePoint(next);
    }
    void refreshPoints() {
        auto const index = pointList->currentRow();
        auto blocker = QSignalBlocker{pointList};
        pointList->clear();
        auto const &s = client.state().navigation;
        for (std::size_t i = 0; i < points.size(); ++i) {
            auto const &p = points[i];
            auto suffix = QString{};
            if (not draftEdited) {
                if (i < s.completed)
                    suffix = QStringLiteral(" · 已到达");
                else if (i + 1 == s.index and s.active)
                    suffix = QStringLiteral(" · 前往中");
                else if (i + 1 == s.index and s.result == "failed")
                    suffix = QStringLiteral(" · 失败");
            }
            pointList->addItem(QStringLiteral("%1  任务点 %1%2\n    %3, %4 m · %5°")
                                   .arg(i + 1)
                                   .arg(suffix)
                                   .arg(p.x, 0, 'f', 2)
                                   .arg(p.y, 0, 'f', 2)
                                   .arg(p.yaw * 180 / pi, 0, 'f', 0));
        }
        pointList->setCurrentRow(std::min(index, static_cast<int>(points.size()) - 1));
    }
    void refreshBoxes() {
        auto blocker = QSignalBlocker{boxList};
        boxList->clear();
        auto const &positions = client.state().scene.mocapPositions;
        if (positions.size() != 3 * obstacleCapacity)
            return;
        for (std::size_t slot = 0; slot < obstacleCapacity; ++slot) {
            if (positions[3 * slot + 2] < 0)
                continue;
            auto *item = new QListWidgetItem(QStringLiteral("障碍物 %1").arg(slot + 1), boxList);
            item->setData(Qt::UserRole, static_cast<unsigned>(slot));
            if (selection == Selection::box and boxSlot == slot)
                boxList->setCurrentItem(item);
        }
        boxLabel->setText(QStringLiteral("场景障碍物 · %1 / 16").arg(boxList->count()));
    }
    void showInspector() {
        auto const has = selection != Selection::none;
        properties->setVisible(has);
        apply->setVisible(has);
        discardButton->setVisible(has);
        yaw->setVisible(selection == Selection::point);
        yawLabel->setVisible(selection == Selection::point);
        sizeLabel->setVisible(selection == Selection::box);
        removeBox->setVisible(selection == Selection::box and not isNew);
        title->setText(
            selection == Selection::none ? QStringLiteral("对象属性")
            : selection == Selection::point
                ? (isNew ? QStringLiteral("新任务点") : QStringLiteral("任务点 %1").arg(pointIndex + 1))
                : (isNew ? QStringLiteral("新障碍物") : QStringLiteral("障碍物 %1").arg(boxSlot + 1)));
        apply->setText(isNew ? (selection == Selection::point ? QStringLiteral("添加到任务")
                                                              : QStringLiteral("放置障碍物"))
                             : QStringLiteral("应用修改"));
    }
    void refresh() {
        client.poll();
        auto const &s = client.state();
        if (s.routeRevision != routeRevision) {
            routeRevision = s.routeRevision;
            points = s.activeRoute;
            draftEdited = false;
            refreshPoints();
            if (selection != Selection::box) {
                choosePoint(points.empty() ? -1 : 0);
                pointList->setCurrentRow(points.empty() ? -1 : 0);
            }
        }
        auto const key = s.navigation.phase + s.navigation.result + std::to_string(s.navigation.index) + "/" +
                         std::to_string(s.navigation.completed);
        if (key != navigationKey) {
            navigationKey = key;
            refreshPoints();
        }
        if (s.scene.revision != sceneRevision or not sceneObserved) {
            sceneRevision = s.scene.revision;
            sceneObserved = not s.scene.positions.empty();
            refreshBoxes();
        }
        if (s.feedback != lastFeedback) {
            lastFeedback = s.feedback;
            feedback->setText(QString::fromStdString(s.feedback));
        }
        if (s.editResponses != editResponses) {
            editResponses = s.editResponses;
            if (s.editApplied and pendingEditSlot and selection == Selection::box and
                boxSlot == *pendingEditSlot) {
                dirty = false;
                isNew = false;
                selection = Selection::none;
                chooseBox(*pendingEditSlot);
                showInspector();
            }
            pendingEditSlot.reset();
        }
        auto const locked = busy();
        start->setEnabled(s.connected and s.navigation.phase == "ready" and not locked and
                          not s.editPending and not points.empty() and not dirty and not isNew);
        stop->setEnabled(s.connected and locked);
        addPoint->setEnabled(not locked and not s.editPending and points.size() < 64);
        addBox->setEnabled(s.connected and not s.editPending and boxList->count() < 16);
        pointList->setEnabled(not s.editPending);
        boxList->setEnabled(not s.editPending);
        selectTool->setEnabled(not s.editPending);
        auto const index = selection == Selection::point and not isNew ? pointList->currentRow() : -1;
        up->setEnabled(not locked and index > 0);
        down->setEnabled(not locked and index >= 0 and index + 1 < static_cast<int>(points.size()));
        removePoint->setEnabled(not locked and index >= 0);
        auto const editable = selection == Selection::point ? not locked : s.connected and not s.editPending;
        properties->setEnabled(editable);
        apply->setEnabled(editable and (dirty or isNew));
        removeBox->setEnabled(s.connected and not s.editPending);
        discardButton->setEnabled(not s.editPending);
        retry->setVisible(s.navigation.result == "failed" and not locked);
        retry->setEnabled(s.connected and not locked and not s.editPending and
                          s.navigation.completed < s.activeRoute.size());
        auto phase = s.navigation.phase == "starting"   ? QStringLiteral("初始化")
                     : s.navigation.phase == "stopping" ? QStringLiteral("正在取消，等待任务结束")
                     : locked
                         ? QStringLiteral("执行中 · %1 / %2").arg(s.navigation.index).arg(s.navigation.count)
                         : QStringLiteral("就绪");
        if (not locked and s.navigation.result == "failed")
            phase += QStringLiteral(" · 任务失败：") + QString::fromStdString(s.navigation.message);
        else if (not locked and s.navigation.result == "succeeded")
            phase += QStringLiteral(" · 任务已完成");
        else if (not locked and s.navigation.result == "canceled")
            phase += QStringLiteral(" · 任务已取消");
        statusLabel->setText(phase);
        connection->setText(s.connected ? QStringLiteral("仿真已连接") : QStringLiteral("连接等待 / 中断"));
        auto overlay = SceneOverlay{points,
                                    locked ? s.path : std::vector<Waypoint>{},
                                    {},
                                    {},
                                    selection == Selection::point and not isNew ? pointIndex : -1};
        if (dirty or isNew) {
            if (selection == Selection::point)
                overlay.pointPreview = Waypoint{x->value(), y->value(), yaw->value() * pi / 180};
            if (selection == Selection::box)
                overlay.boxPreview = ObstacleEdit{boxSlot, true, x->value(), y->value()};
        }
        view->present(s.scene, overlay);
    }
    void closeEvent(QCloseEvent *event) override {
        client.cancel(); // run.py remains the supervisor and drains before stopping bridge.
        QMainWindow::closeEvent(event);
    }
    GuiClient &client;
    std::vector<Waypoint> points{{1, -1.2, 0}, {4, -1.2, pi / 2}, {4, 1.5, pi}};
    Selection selection = Selection::none;
    int pointIndex = -1;
    std::size_t boxSlot = 0;
    std::optional<std::size_t> pendingEditSlot;
    bool isNew = false, dirty = false, draftEdited = true, sceneObserved = false;
    std::uint64_t routeRevision = 0, sceneRevision = 0, editResponses = 0;
    std::string lastFeedback, navigationKey;
    SceneView *view = nullptr;
    QListWidget *pointList = nullptr, *boxList = nullptr;
    QLabel *title = nullptr, *sizeLabel = nullptr, *feedback = nullptr, *statusLabel = nullptr,
           *connection = nullptr, *boxLabel = nullptr;
    QWidget *properties = nullptr, *yawLabel = nullptr;
    QDoubleSpinBox *x = nullptr, *y = nullptr, *yaw = nullptr;
    QPushButton *selectTool = nullptr, *addPoint = nullptr, *addBox = nullptr, *start = nullptr,
                *stop = nullptr;
    QPushButton *up = nullptr, *down = nullptr, *removePoint = nullptr, *removeBox = nullptr,
                *apply = nullptr, *discardButton = nullptr, *retry = nullptr, *top = nullptr,
                *orbit = nullptr;
};
} // namespace
std::unique_ptr<QMainWindow> makeWorkbench(GuiClient &client, std::string const &modelFile) {
    return std::make_unique<Workbench>(client, modelFile);
}
} // namespace simulation
