"""Qt controls for the persistent lrclexec navigation session."""

import signal
import sys
import time

import rclpy
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QApplication,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from session_client import SessionClient


class Controls(QWidget):
    def __init__(self, client):
        super().__init__()
        self.client = client
        self.pending = None
        self.requested_at = 0.0
        self.setWindowTitle("lrclexec 导航控制")
        self.setMinimumWidth(380)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(14)
        help_text = QLabel(
            "在 RViz 选择 2D Goal Pose，在地图上按住并拖动：\n"
            "位置决定目的地，箭头决定最终朝向。\n"
            "再次设置目标会停止旧任务，然后前往最新目标。"
        )
        help_text.setWordWrap(True)
        layout.addWidget(help_text)
        self.state = QLabel("等待导航就绪…")
        self.state.setStyleSheet("font-size: 18px; font-weight: bold;")
        layout.addWidget(self.state)
        self.details = QLabel("定位与导航服务正在启动")
        self.details.setWordWrap(True)
        self.details.setTextFormat(Qt.PlainText)
        layout.addWidget(self.details)
        self.cancel_button = QPushButton("取消当前目标")
        self.cancel_button.setMinimumHeight(38)
        self.cancel_button.setEnabled(False)
        self.cancel_button.clicked.connect(self.cancel_goal)
        layout.addWidget(self.cancel_button)
        self.response = QLabel("")
        self.response.setWordWrap(True)
        self.response.setTextFormat(Qt.PlainText)
        layout.addWidget(self.response)
        close_button = QPushButton("结束仿真")
        close_button.setMinimumHeight(38)
        close_button.clicked.connect(self.close)
        layout.addWidget(close_button)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)

    def cancel_goal(self):
        self.pending = self.client.request_cancel()
        self.requested_at = time.monotonic()
        self.cancel_button.setEnabled(False)
        self.response.setText("正在请求取消…")

    def tick(self):
        rclpy.spin_once(self.client, timeout_sec=0)
        if self.pending is not None:
            if self.pending.done():
                try:
                    response = self.pending.result()
                    self.response.setText(
                        "已请求取消，正在等待任务结束。"
                        if response.success
                        else "当前没有执行或等待中的目标。"
                    )
                except Exception as error:  # noqa: BLE001 - Keep controls usable.
                    self.response.setText(f"取消请求失败：{error}")
                self.pending = None
            elif time.monotonic() - self.requested_at > 3:
                self.client.cancel.remove_pending_request(self.pending)
                self.pending = None
                self.response.setText("取消请求应答超时，请检查任务状态。")
        status = self.client.status
        if not status:
            return
        fresh = time.monotonic() - self.client.updated_at < 3
        phase = status["state"]
        self.state.setText(
            {
                "starting": "正在初始化定位与导航",
                "ready": "就绪 · 可设置新目标",
                "running": "正在导航",
                "stopping": "正在停止并排空任务",
                "closed": "会话已结束",
            }.get(phase, phase)
            if fresh
            else "导航通信中断"
        )
        result = {"succeeded": "已到达", "canceled": "已取消", "failed": "失败"}
        if status["active"]:
            detail = f"当前目标 #{status['active']}"
            if status["pending"]:
                detail += f"；等待替换为 #{status['pending']}"
        elif status["last_id"]:
            detail = (
                f"目标 #{status['last_id']} {result.get(status['last_result'], '')}"
            )
        else:
            detail = "等待在 RViz 中设置目标"
        if status["message"]:
            detail += "\n" + status["message"]
        self.details.setText(detail)
        self.cancel_button.setEnabled(
            fresh
            and bool(status["active"] or status["pending"])
            and self.pending is None
            and self.client.cancel.service_is_ready()
        )
        if phase == "ready" and self.pending is None:
            self.response.clear()


def main():
    app = QApplication(sys.argv)
    rclpy.init(args=[])
    client = SessionClient()
    window = Controls(client)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    window.show()
    try:
        return app.exec_()
    finally:
        window.timer.stop()
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
