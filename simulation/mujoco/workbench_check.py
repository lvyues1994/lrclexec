"""Verify the workbench ROS contracts against real MuJoCo and Nav2."""

import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from session_client import SessionClient
from std_msgs.msg import String
from visualization_msgs.msg import Marker


def run(client):
    latest = {}
    paths = []
    peak = 0
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = []

    def observe(name, msg):
        latest[name] = json.loads(msg.data) if name in ("scene", "ack") else msg

    for name, kind, topic, qos in (
        ("scene", String, "simulation/scene", durable),
        ("ack", String, "simulation/edit_result", 32),
        ("odom", Odometry, "odom", 10),
        ("map", OccupancyGrid, "map", durable),
        ("scan", LaserScan, "scan", qos_profile_sensor_data),
    ):
        subscriptions.append(
            client.create_subscription(
                kind, topic, lambda msg, key=name: observe(key, msg), qos
            )
        )

    def path(msg):
        paths.append((time.monotonic(), msg))
        del paths[:-3]

    def controllers(msg):
        nonlocal peak
        peak = max(
            peak,
            sum(
                row.status in (GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)
                for row in msg.status_list
            ),
        )

    subscriptions.append(
        client.create_subscription(Path, "navigation/path", path, durable)
    )
    subscriptions.append(
        client.create_subscription(
            GoalStatusArray, "follow_path/_action/status", controllers, durable
        )
    )
    edits = client.create_publisher(Marker, "simulation/edit_obstacle", 1)
    costmaps = client.create_client(GetCostmap, "global_costmap/get_costmap")
    request = 0

    def wait(predicate, description, timeout=40):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(client, timeout_sec=0.05)
            if peak > 1:
                raise RuntimeError("overlapping executing/canceling FollowPath goals")
            if predicate():
                print(description, json.dumps(client.status), flush=True)
                return
        raise RuntimeError(f"timeout: {description}; status={client.status}")

    def cost_at_box(x, y):
        future = costmaps.call_async(GetCostmap.Request())
        wait(future.done, "costmap received", 5)
        grid = future.result().map
        m = grid.metadata
        cells = []
        for px in (x - 0.4, x, x + 0.4):
            for py in (y - 0.3, y, y + 0.3):
                col = int((px - m.origin.position.x) / m.resolution)
                row = int((py - m.origin.position.y) / m.resolution)
                if 0 <= col < m.size_x and 0 <= row < m.size_y:
                    cells.append(grid.data[row * m.size_x + col])
        return max(cells, default=255)

    def await_cost(x, y, occupied):
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            value = cost_at_box(x, y)
            if (value >= 253) == occupied:
                print(
                    "costmap occupied" if occupied else "costmap cleared",
                    value,
                    flush=True,
                )
                return
            until = time.monotonic() + 0.25
            wait(lambda deadline=until: time.monotonic() >= deadline, "next clearing scan", 2)
        raise RuntimeError(f"costmap did not become occupied={occupied}: {value}")

    def edit(x, y, *, active=True, slot=0, expected=True, frame="map"):
        nonlocal request
        request += 1
        msg = Marker()
        msg.header.frame_id, msg.header.stamp.nanosec = frame, request
        msg.ns, msg.id, msg.type = "workbench_check", slot, Marker.CUBE
        msg.action = Marker.ADD if active else Marker.DELETE
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
            float(x),
            float(y),
            0.3,
        )
        msg.pose.orientation.w = 1.0
        msg.scale.x, msg.scale.y, msg.scale.z = 0.8, 0.6, 0.6
        old_revision = latest["scene"]["revision"]
        edits.publish(msg)
        wait(
            lambda: latest.get("ack", {}).get("request_nanosec") == request,
            "obstacle command acknowledged",
            5,
        )
        ack = latest["ack"]
        if ack["applied"] != expected:
            raise RuntimeError(f"unexpected obstacle result: {ack}")
        if not expected:
            if ack["revision"] != old_revision:
                raise RuntimeError("rejected edit changed scene")
            return
        wait(
            lambda: latest["scene"]["revision"] == ack["revision"],
            "confirmed scene received",
            5,
        )
        xyz = latest["scene"]["mocap_pos"][3 * slot : 3 * slot + 3]
        if active and (abs(xyz[0] - x) > 1e-8 or abs(xyz[1] - y) > 1e-8):
            raise RuntimeError("scene disagrees with accepted obstacle position")
        if not active and xyz[2] >= 0:
            raise RuntimeError("deleted obstacle still active")

    def route(points, expected="succeeded", timeout=60):
        accepted = client.status["accepted"]
        client.send_route(points)
        wait(lambda: client.status["accepted"] > accepted, "route accepted", 5)
        identity = client.status["accepted"]
        wait(
            lambda: (
                client.status["last_id"] == identity
                and client.status["state"] == "ready"
            ),
            "route drained",
            timeout,
        )
        if client.status["last_result"] != expected:
            raise RuntimeError(f"route result mismatch: {client.status}")

    wait(
        lambda: (
            client.status.get("state") == "ready"
            and all(key in latest for key in ("scene", "scan", "odom", "map"))
            and client.routes.get_subscription_count()
            and edits.get_subscription_count()
            and costmaps.service_is_ready()
        ),
        "workbench ready",
    )
    original_map = list(latest["map"].data)
    edit(0, 0, expected=False)  # Robot footprint.
    edit(1, 1, frame="odom", expected=False)
    edit(0.9, 1.5)
    await_cost(0.9, 1.5, True)
    edit(0.9, 1.5, active=False)
    await_cost(0.9, 1.5, False)
    route([(0.6, -0.5, 0), (1.0, -0.5, math.pi / 2), (0.8, 0, 0)])
    if client.status["waypoint_completed"] != 3:
        raise RuntimeError("three-point route did not visit every waypoint")
    route([(1.0, 0, 0), (20.0, 0, 0), (0.0, 0, 0)], expected="failed", timeout=25)
    if client.status["waypoint_index"] != 2 or client.status["waypoint_completed"] != 1:
        raise RuntimeError("failure did not stop at the second waypoint")
    accepted = client.status["accepted"]
    client.send_route([(4.0, 0, 0)])
    wait(lambda: client.status["accepted"] > accepted, "moving route accepted", 5)
    identity = client.status["accepted"]
    wait(
        lambda: (
            client.status["active"] == identity
            and paths
            and paths[-1][1].poses
            and abs(latest["odom"].twist.twist.linear.x) > 0.05
        ),
        "robot moving on planned route",
        30,
    )
    candidate = next(
        (p.pose.position for p in paths[-1][1].poses if 2.9 < p.pose.position.x < 3.1),
        None,
    )
    if candidate is None:
        raise RuntimeError("no forward path location for dynamic obstacle")
    x, y = candidate.x, candidate.y
    p = latest["odom"].pose.pose.position
    if math.hypot(x - p.x, y - p.y) < 1.3:
        raise RuntimeError("robot too close for dynamic obstacle insertion")
    edit(x, y)
    applied_at = time.monotonic()
    await_cost(x, y, True)

    def avoids_box(path):
        return bool(path.poses) and all(
            not (
                abs(p.pose.position.x - x) < 0.64 and abs(p.pose.position.y - y) < 0.54
            )
            for p in path.poses
        )

    wait(
        lambda: any(received >= applied_at and avoids_box(p) for received, p in paths),
        "new path avoids the inserted obstacle",
        12,
    )
    wait(
        lambda: (
            client.status["last_id"] == identity
            and client.status["last_result"] == "succeeded"
        ),
        "dynamic obstacle route succeeded",
        70,
    )
    p = latest["odom"].pose.pose.position
    if math.hypot(p.x - 4, p.y) > 0.15:
        raise RuntimeError("successful action did not reach physical target")
    edit(x, y, active=False)
    await_cost(x, y, False)
    if list(latest["map"].data) != original_map:
        raise RuntimeError("obstacle edits changed the static map")
    accepted = client.status["accepted"]
    client.send_route([(1.0, 1.5, 0), (0.0, 0, 0)])
    wait(
        lambda: (
            client.status["accepted"] > accepted and client.status["state"] == "running"
        ),
        "route active before cancel",
        5,
    )
    canceled = client.status["canceled"]
    response = client.request_cancel()
    wait(response.done, "cancel response", 5)
    wait(
        lambda: (
            client.status["canceled"] > canceled and client.status["state"] == "ready"
        ),
        "route canceled and drained",
        10,
    )
    if client.status["waypoint_completed"] or client.status["waypoint_index"] != 1:
        raise RuntimeError("canceled route started a remaining waypoint")
    client.send_route([(0.0, 1.5, 0), (0.0, 0, 0)])
    wait(
        lambda: (
            client.status["state"] == "running"
            and abs(latest["odom"].twist.twist.linear.x) > 0.05
        ),
        "moving before workbench close",
        30,
    )
    if peak != 1:
        raise RuntimeError(f"expected one observed active FollowPath, got {peak}")
    print(
        "SESSION_CHECK_PASSED "
        + json.dumps({"peak_controllers": peak, **client.status}),
        flush=True,
    )


if __name__ == "__main__":
    rclpy.init()
    node = SessionClient()
    try:
        run(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
