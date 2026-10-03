"""Drive the same goal topic and cancel service used by RViz and Qt."""

import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from session_client import SessionClient


def run(client):
    odometry = []
    peak = 0

    def pose(message):
        odometry.append(message)
        del odometry[:-3]

    def controllers(message):
        nonlocal peak
        peak = max(
            peak,
            sum(
                item.status
                in (GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)
                for item in message.status_list
            ),
        )

    odom_sub = client.create_subscription(Odometry, "odom", pose, 10)
    action_sub = client.create_subscription(
        GoalStatusArray,
        "follow_path/_action/status",
        controllers,
        QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )

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

    def reached(x, y, yaw):
        if not odometry:
            return False
        p = odometry[-1].pose.pose
        q = p.orientation
        actual = math.atan2(
            2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
        )
        return (
            math.hypot(p.position.x - x, p.position.y - y) < 0.15
            and abs(math.remainder(actual - yaw, 2 * math.pi)) < 0.18
        )

    def go(x, y, yaw=0.0):
        succeeded = client.status["succeeded"]
        client.send_goal(x, y, yaw)
        wait(lambda: client.status["succeeded"] > succeeded, "target succeeded")
        wait(lambda: reached(x, y, yaw), "physical target matched", 5)

    def motion_goal(description):
        accepted = client.status["accepted"]
        previous_stamp = odometry[-1].header.stamp
        previous_time = previous_stamp.sec * 1_000_000_000 + previous_stamp.nanosec
        client.send_goal(4.0, 0.0)
        wait(lambda: client.status["accepted"] > accepted, "motion target accepted", 5)
        goal_id = client.status["accepted"]
        wait(lambda: client.status["active"] == goal_id, "motion target started", 5)
        odometry.clear()
        wait(
            lambda: (
                len(odometry) == 3
                and client.status["active"] == goal_id
                and all(
                    row.header.stamp.sec * 1_000_000_000 + row.header.stamp.nanosec
                    > previous_time
                    and abs(row.twist.twist.linear.x) > 0.05
                    for row in odometry
                )
            ),
            description,
        )

    wait(
        lambda: (
            client.status.get("state") == "ready"
            and client.goals.get_subscription_count() > 0
            and client.cancel.service_is_ready()
        ),
        "session ready",
    )
    go(0.8, 0.0)
    go(1.1, 0.6, math.pi / 2)
    motion_goal("moving before UI cancellation")
    canceled = client.status["canceled"]
    response = client.request_cancel()
    wait(response.done, "cancel service responded", 5)
    if not response.result().success:
        raise RuntimeError("cancel service rejected active target")
    wait(
        lambda: (
            client.status["canceled"] > canceled and client.status["state"] == "ready"
        ),
        "canceled and drained",
        10,
    )
    odometry.clear()
    wait(
        lambda: (
            len(odometry) == 3
            and all(
                abs(row.twist.twist.linear.x) < 0.03
                and abs(row.twist.twist.angular.z) < 0.08
                for row in odometry
            )
        ),
        "physically stopped after cancel",
        5,
    )

    rejected = client.status["rejected"]
    client.send_goal(0.5, 0.0, frame="odom")
    wait(lambda: client.status["rejected"] > rejected, "invalid frame rejected", 5)
    failed = client.status["failed"]
    client.send_goal(20.0, 0.0)
    wait(lambda: client.status["failed"] > failed, "unreachable target failed", 10)
    go(1.0, 0.5)  # The same session survives both cancellation and planner failure.

    motion_goal("moving before replacement")
    accepted = client.status["accepted"]
    succeeded = client.status["succeeded"]
    client.send_goal(0.6, 0.5)
    wait(lambda: client.status["accepted"] > accepted, "replacement accepted", 5)
    client.send_goal(0.4, 0.3)
    wait(lambda: client.status["accepted"] > accepted + 1, "latest target accepted", 5)
    latest = client.status["accepted"]
    wait(
        lambda: (
            client.status["succeeded"] > succeeded
            and client.status["last_id"] == latest
            and client.status["last_result"] == "succeeded"
            and client.status["state"] == "ready"
            and not client.status["active"]
            and not client.status["pending"]
            and reached(0.4, 0.3, 0.0)
        ),
        "latest target succeeded",
    )
    if peak != 1:
        raise RuntimeError("no live FollowPath status observed")
    motion_goal("moving before launcher shutdown")
    print(
        "SESSION_CHECK_PASSED "
        + json.dumps({"peak_controllers": peak, **client.status}),
        flush=True,
    )
    # Returning emulates closing the Qt window. The launcher must now stop and
    # drain the live mission, verify physical rest, then shut down the stack.
    client.destroy_subscription(odom_sub)
    client.destroy_subscription(action_sub)


if __name__ == "__main__":
    rclpy.init()
    node = SessionClient()
    try:
        run(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
