"""ROS boundary shared by the headless session and workbench checks."""

import json
import math
import time

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger


class SessionClient(Node):
    def __init__(self):
        super().__init__("lrclexec_navigation_controls")
        self.status = {}
        self.updated_at = 0.0
        self.goals = self.create_publisher(PoseStamped, "goal_pose", 10)
        self.routes = self.create_publisher(PoseArray, "navigation/route", 1)
        self.cancel = self.create_client(Trigger, "navigation/cancel")
        self.subscription = self.create_subscription(
            String,
            "navigation/status",
            self.observe,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

    def observe(self, message):
        self.status = json.loads(message.data)
        self.updated_at = time.monotonic()

    def send_goal(self, x, y, yaw=0.0, *, frame="map"):
        message = PoseStamped()
        message.header.frame_id = frame
        # The session stamps accepted targets with its simulation clock.
        message.pose.position.x = float(x)
        message.pose.position.y = float(y)
        message.pose.orientation.z = math.sin(yaw / 2)
        message.pose.orientation.w = math.cos(yaw / 2)
        self.goals.publish(message)

    def request_cancel(self):
        return self.cancel.call_async(Trigger.Request())

    def send_route(self, points, *, frame="map"):
        message = PoseArray()
        message.header.frame_id = frame
        for x, y, yaw in points:
            pose = Pose()
            pose.position.x, pose.position.y = float(x), float(y)
            pose.orientation.z, pose.orientation.w = (
                math.sin(yaw / 2),
                math.cos(yaw / 2),
            )
            message.poses.append(pose)
        self.routes.publish(message)
