#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge


class PareDetector(Node):
    def __init__(self):
        super().__init__("pare_detector")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("pare_topic", "/pare_detectado")
        self.declare_parameter("debug_topic", "/pare/debug_image")

        self.declare_parameter("attention_only", False)
        self.declare_parameter("min_area", 200)
        self.declare_parameter("max_area", 250000)
        self.declare_parameter("min_ratio", 0.25)
        self.declare_parameter("max_ratio", 3.50)

        image_topic = self.get_parameter("image_topic").value
        pare_topic = self.get_parameter("pare_topic").value
        debug_topic = self.get_parameter("debug_topic").value

        self.bridge = CvBridge()
        self.atencion_pare = True

        self.pub_pare = self.create_publisher(Bool, pare_topic, 10)
        self.pub_debug = self.create_publisher(Image, debug_topic, 10)

        self.create_subscription(Image, image_topic, self.image_cb, 10)
        self.create_subscription(Bool, "/maze/atencion_pare", self.att_cb, 10)

        self.get_logger().info("pare_detector listo: detectando rojo/PARE.")

    def att_cb(self, msg):
        self.atencion_pare = bool(msg.data)

    def publish_bool(self, value):
        msg = Bool()
        msg.data = bool(value)
        self.pub_pare.publish(msg)

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"No pude convertir imagen: {e}")
            self.publish_bool(False)
            return

        attention_only = bool(self.get_parameter("attention_only").value)

        if attention_only and not self.atencion_pare:
            self.publish_bool(False)
            self.publish_debug(frame, False, None)
            return

        detected, best_box, mask = self.detect_pare(frame)
        self.publish_bool(detected)
        self.publish_debug(frame, detected, best_box, mask)

    def detect_pare(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 60, 45])
        upper_red1 = np.array([16, 255, 255])

        lower_red2 = np.array([158, 60, 45])
        upper_red2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        min_area = float(self.get_parameter("min_area").value)
        max_area = float(self.get_parameter("max_area").value)
        min_ratio = float(self.get_parameter("min_ratio").value)
        max_ratio = float(self.get_parameter("max_ratio").value)

        best_area = 0
        best_box = None

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area < min_area or area > max_area:
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            if h <= 0:
                continue

            ratio = w / float(h)

            if ratio < min_ratio or ratio > max_ratio:
                continue

            if area > best_area:
                best_area = area
                best_box = (x, y, w, h, area, ratio)

        return best_box is not None, best_box, mask

    def publish_debug(self, frame, detected, best_box, mask=None):
        debug = frame.copy()

        label = "PARE: SI" if detected else "PARE: NO"

        if detected and best_box is not None:
            x, y, w, h, area, ratio = best_box
            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 3)
            cv2.putText(
                debug,
                f"area={int(area)} ratio={ratio:.2f}",
                (x, max(25, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

        cv2.putText(
            debug,
            label,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0) if detected else (0, 0, 255),
            2
        )

        try:
            msg = self.bridge.cv2_to_imgmsg(debug, "bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_debug.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"No pude publicar debug: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PareDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
