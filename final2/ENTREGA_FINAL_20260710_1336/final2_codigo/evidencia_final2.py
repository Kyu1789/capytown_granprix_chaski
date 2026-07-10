#!/usr/bin/env python3
# evidencia_final2.py
# Monitor PASIVO: NO mueve el robot.
# Escucha /odom_raw, /scan y /pare_detectado.
# Publica en RViz: /final2/path y /final2/markers.
# Al cerrar con CTRL+C guarda CSV + SVG en /root/final2/evidencias.

import os
import csv
import math
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


def dist2(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class EvidenciaFinal2(Node):
    def __init__(self):
        super().__init__("evidencia_final2")

        self.outdir = "/root/final2/evidencias"
        os.makedirs(self.outdir, exist_ok=True)

        self.points = []
        self.intersections = []
        self.pares = []

        self.last_xy = None
        self.last_inter_xy = None
        self.last_pare_t = 0.0

        self.front = float("inf")
        self.left = float("inf")
        self.right = float("inf")

        self.path = Path()
        self.path.header.frame_id = "odom"

        self.path_pub = self.create_publisher(Path, "/final2/path", 10)
        self.mark_pub = self.create_publisher(MarkerArray, "/final2/markers", 10)

        self.create_subscription(Odometry, "/odom_raw", self.odom_cb, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.create_subscription(Bool, "/pare_detectado", self.pare_cb, 10)

        self.timer = self.create_timer(0.3, self.publish_all)

        self.get_logger().info("EVIDENCIA FINAL2 lista. NO mueve el robot.")
        self.get_logger().info("RViz: Fixed Frame odom | Path /final2/path | MarkerArray /final2/markers | LaserScan /scan")

    def sector(self, msg, center_deg, width_deg):
        vals = []
        a0 = math.radians(center_deg - width_deg / 2.0)
        a1 = math.radians(center_deg + width_deg / 2.0)

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r <= msg.range_min or r >= msg.range_max:
                continue

            a = msg.angle_min + i * msg.angle_increment
            if a0 <= a <= a1:
                vals.append(r)

        if not vals:
            return float("inf")

        vals.sort()
        return vals[int(0.25 * (len(vals) - 1))]

    def scan_cb(self, msg):
        self.front = self.sector(msg, 0, 25)
        self.left = self.sector(msg, 90, 40)
        self.right = self.sector(msg, -90, 40)

        if self.last_xy is None:
            return

        # Marca intersecciones/aperturas aproximadas.
        side_open = 0.72
        hay_apertura = self.left > side_open or self.right > side_open

        if hay_apertura:
            if self.last_inter_xy is None or dist2(self.last_xy, self.last_inter_xy) > 0.28:
                self.intersections.append((self.last_xy[0], self.last_xy[1], time.time()))
                self.last_inter_xy = self.last_xy
                self.get_logger().info(
                    f"Marcador interseccion/apertura: F={self.front:.2f} L={self.left:.2f} R={self.right:.2f}"
                )

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        xy = (x, y)

        if self.last_xy is None or dist2(xy, self.last_xy) > 0.015:
            self.points.append(xy)
            self.last_xy = xy

            ps = PoseStamped()
            ps.header = msg.header
            ps.header.frame_id = "odom"
            ps.pose = msg.pose.pose

            self.path.header.stamp = msg.header.stamp
            self.path.poses.append(ps)

    def pare_cb(self, msg):
        if not msg.data or self.last_xy is None:
            return

        now = time.time()
        if now - self.last_pare_t > 2.0:
            self.pares.append((self.last_xy[0], self.last_xy[1], now))
            self.last_pare_t = now
            self.get_logger().warn("Marcador PARE detectado.")

    def make_marker(self, mid, typ, ns, r, g, b, a=1.0):
        m = Marker()
        m.header.frame_id = "odom"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = typ
        m.action = Marker.ADD
        m.color.r = float(r)
        m.color.g = float(g)
        m.color.b = float(b)
        m.color.a = float(a)
        return m

    def publish_all(self):
        self.path_pub.publish(self.path)

        arr = MarkerArray()
        mid = 0

        if len(self.points) >= 2:
            line = self.make_marker(mid, Marker.LINE_STRIP, "trayectoria", 0.0, 0.7, 1.0, 1.0)
            mid += 1
            line.scale.x = 0.025
            for x, y in self.points:
                p = Point()
                p.x = float(x)
                p.y = float(y)
                p.z = 0.03
                line.points.append(p)
            arr.markers.append(line)

        for x, y, _ in self.intersections:
            m = self.make_marker(mid, Marker.SPHERE, "intersecciones", 1.0, 0.6, 0.0, 0.9)
            mid += 1
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 0.08
            m.scale.x = 0.12
            m.scale.y = 0.12
            m.scale.z = 0.12
            arr.markers.append(m)

        for x, y, _ in self.pares:
            m = self.make_marker(mid, Marker.CUBE, "pare", 1.0, 0.0, 0.0, 1.0)
            mid += 1
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 0.12
            m.scale.x = 0.16
            m.scale.y = 0.16
            m.scale.z = 0.16
            arr.markers.append(m)

        self.mark_pub.publish(arr)

    def save_files(self):
        csv_path = os.path.join(self.outdir, "trayectoria_final2.csv")
        svg_path = os.path.join(self.outdir, "grafico_trayectoria_final2.svg")

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["tipo", "x", "y", "t"])
            for x, y in self.points:
                writer.writerow(["odom", x, y, ""])
            for x, y, t in self.intersections:
                writer.writerow(["interseccion", x, y, t])
            for x, y, t in self.pares:
                writer.writerow(["pare", x, y, t])

        self.save_svg(svg_path)

        self.get_logger().info(f"Guardado CSV: {csv_path}")
        self.get_logger().info(f"Guardado SVG: {svg_path}")

    def save_svg(self, path):
        pts = self.points[:]
        if not pts:
            pts = [(0.0, 0.0)]

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        minx, maxx = min(xs), max(xs)
        miny, maxy = min(ys), max(ys)

        W, H = 900, 650
        pad = 60
        sx = (W - 2 * pad) / max(0.1, maxx - minx)
        sy = (H - 2 * pad) / max(0.1, maxy - miny)
        scale = min(sx, sy)

        def tr(x, y):
            X = pad + (x - minx) * scale
            Y = H - pad - (y - miny) * scale
            return X, Y

        poly = " ".join(f"{tr(x, y)[0]:.1f},{tr(x, y)[1]:.1f}" for x, y in self.points)

        with open(path, "w") as f:
            f.write(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">\n')
            f.write('<rect width="100%" height="100%" fill="#f8fafc"/>\n')
            f.write('<text x="30" y="35" font-size="24" font-family="Arial" fill="#0f766e">FINAL2 - trayectoria / odometria + eventos</text>\n')
            f.write('<text x="30" y="60" font-size="14" font-family="Arial" fill="#475569">Azul: trayectoria | Naranja: intersecciones/aperturas | Rojo: PARE detectado</text>\n')

            if poly:
                f.write(f'<polyline points="{poly}" fill="none" stroke="#0284c7" stroke-width="5" stroke-linejoin="round" stroke-linecap="round"/>\n')

            for x, y, _ in self.intersections:
                X, Y = tr(x, y)
                f.write(f'<circle cx="{X:.1f}" cy="{Y:.1f}" r="10" fill="#f59e0b" opacity="0.9"/>\n')

            for x, y, _ in self.pares:
                X, Y = tr(x, y)
                f.write(f'<rect x="{X - 10:.1f}" y="{Y - 10:.1f}" width="20" height="20" fill="#dc2626" opacity="0.95"/>\n')
                f.write(f'<text x="{X + 13:.1f}" y="{Y + 5:.1f}" font-size="12" font-family="Arial" fill="#991b1b">PARE</text>\n')

            f.write('</svg>\n')


def main():
    rclpy.init()
    node = EvidenciaFinal2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_files()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
