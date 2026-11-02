#!/usr/bin/env python3
"""Display-only leaf TF anchors and compact marker labels. No control output."""
import re
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

rclpy.init()
node = rclpy.create_node("relocalization_display_helper")
qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
pub = node.create_publisher(
    MarkerArray, "/global_relocalization/visual/readable_markers", qos
)
tf = StaticTransformBroadcaster(node)
anchors = []
for parent in ("map", "relocalization_reference"):
    t = TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = node.get_clock().now().to_msg()
    t.child_frame_id = parent + "_display_anchor"
    t.transform.rotation.w = 1.0
    anchors.append(t)
tf.sendTransform(anchors)
last = None


def update(message):
    global last
    out = MarkerArray()
    for m in message.markers:
        if m.action == Marker.DELETEALL:
            out.markers.append(m)
            continue
        if m.type == Marker.TEXT_VIEW_FACING:
            if m.id >= 1000 and m.id != 1900:
                continue
            if m.id == 1:
                full = m.text.split(" ", 1)[-1]
                m.text = "GLOBAL RELOCALIZATION: " + full.split(":", 1)[0]
                m.pose.position.x = 0.0
                m.pose.position.y = 11.0
                m.pose.position.z = 0.5
                m.scale.z = 0.45
            elif m.id == 0:
                text = m.text.split("\n")
                detail = text[1] if len(text) > 1 else m.text
                event = detail.split(" index=")[0]
                fields = []
                for pattern in (
                    r"index=([^ ]+)",
                    r"SC#([^ ]+)",
                    r"RMSE=([^ ]+)",
                    r"ratio=([^ ]+)",
                    r"consistent=([^ ]+)",
                ):
                    found = re.search(pattern, detail)
                    if found:
                        fields.append(found.group(0))
                m.text = event + "\n" + "   ".join(fields)
                m.pose.position.x = 0.0
                m.pose.position.y = 9.5
                m.pose.position.z = 0.5
                m.scale.z = 0.28
            elif m.id == 1900:
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 0.0
                m.scale.z = 0.38
                m.pose.position.z += 0.6
        out.markers.append(m)
    last = out
    pub.publish(out)


node.create_timer(1.0, lambda: pub.publish(last) if last is not None else None)
node.create_subscription(
    MarkerArray, "/global_relocalization/visual/markers", update, qos
)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
