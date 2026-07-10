#!/usr/bin/env python3
# live_plot_final2.py
# Monitor PASIVO: NO mueve el robot.
# Muestra grafico EN VIVO con matplotlib:
# - Izquierda: trayectoria /odom_raw + marcas de intersecciones + PARE
# - Derecha: radar LiDAR /scan
# Tambien guarda PNG/SVG/CSV al cerrar con CTRL+C.

import os
import csv
import math
import time
import threading

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


def dist2(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class LiveData(Node):
    def __init__(self):
        super().__init__("live_plot_final2")

        self.outdir = "/root/final2/evidencias"
        os.makedirs(self.outdir, exist_ok=True)

        self.lock = threading.Lock()

        self.points = []
        self.intersections = []
        self.pares = []

        self.last_xy = None
        self.last_inter_xy = None
        self.last_pare_t = 0.0

        self.front = float("inf")
        self.left = float("inf")
        self.right = float("inf")

        self.scan_angles = []
        self.scan_ranges = []

        self.create_subscription(Odometry, "/odom_raw", self.odom_cb, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.create_subscription(Bool, "/pare_detectado", self.pare_cb, 10)

        self.get_logger().info("LIVE PLOT FINAL2 listo. NO mueve el robot.")

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
        angles = []
        ranges = []

        # Guardamos puntos para radar. Limitamos rango para que se vea bonito.
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r <= msg.range_min or r >= msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment
            angles.append(a)
            ranges.append(min(float(r), 3.0))

        front = self.sector(msg, 0, 25)
        left = self.sector(msg, 90, 40)
        right = self.sector(msg, -90, 40)

        with self.lock:
            self.scan_angles = angles
            self.scan_ranges = ranges
            self.front = front
            self.left = left
            self.right = right

            if self.last_xy is not None:
                side_open = 0.72
                hay_apertura = left > side_open or right > side_open
                if hay_apertura:
                    if self.last_inter_xy is None or dist2(self.last_xy, self.last_inter_xy) > 0.28:
                        self.intersections.append((self.last_xy[0], self.last_xy[1], time.time()))
                        self.last_inter_xy = self.last_xy
                        self.get_logger().info(
                            f"Interseccion/apertura: F={front:.2f} L={left:.2f} R={right:.2f}"
                        )

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        xy = (x, y)

        with self.lock:
            if self.last_xy is None or dist2(xy, self.last_xy) > 0.015:
                self.points.append(xy)
                self.last_xy = xy

    def pare_cb(self, msg):
        if not msg.data:
            return

        now = time.time()
        with self.lock:
            if self.last_xy is None:
                return
            if now - self.last_pare_t > 2.0:
                self.pares.append((self.last_xy[0], self.last_xy[1], now))
                self.last_pare_t = now
                self.get_logger().warn("PARE detectado y marcado.")

    def snapshot(self):
        with self.lock:
            return {
                "points": list(self.points),
                "intersections": list(self.intersections),
                "pares": list(self.pares),
                "scan_angles": list(self.scan_angles),
                "scan_ranges": list(self.scan_ranges),
                "front": self.front,
                "left": self.left,
                "right": self.right,
            }

    def save_csv(self):
        path = os.path.join(self.outdir, "live_plot_final2_trayectoria.csv")
        snap = self.snapshot()
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["tipo", "x", "y", "t"])
            for x, y in snap["points"]:
                writer.writerow(["odom", x, y, ""])
            for x, y, t in snap["intersections"]:
                writer.writerow(["interseccion", x, y, t])
            for x, y, t in snap["pares"]:
                writer.writerow(["pare", x, y, t])
        self.get_logger().info(f"CSV guardado: {path}")


def ros_spin_thread(node):
    rclpy.spin(node)


def run_plot(node):
    import matplotlib
    import matplotlib.pyplot as plt

    # Intentar modo interactivo. Si no hay display, guardara imagen repetidamente.
    plt.ion()

    fig = plt.figure(figsize=(12, 6))
    fig.canvas.manager.set_window_title("FINAL2 - Live Plot / LiDAR")
    ax_map = fig.add_subplot(1, 2, 1)
    ax_scan = fig.add_subplot(1, 2, 2, projection="polar")

    last_save = 0.0

    while rclpy.ok():
        snap = node.snapshot()
        pts = snap["points"]
        inters = snap["intersections"]
        pares = snap["pares"]

        ax_map.clear()
        ax_scan.clear()

        ax_map.set_title("Trayectoria /odom_raw + eventos")
        ax_map.set_xlabel("x")
        ax_map.set_ylabel("y")
        ax_map.grid(True)

        if len(pts) >= 2:
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            ax_map.plot(xs, ys, linewidth=2, label="trayectoria")
            ax_map.scatter(xs[-1], ys[-1], s=70, marker="o", label="robot ahora")

            # Autoescala amable.
            margin = 0.25
            ax_map.set_xlim(min(xs) - margin, max(xs) + margin)
            ax_map.set_ylim(min(ys) - margin, max(ys) + margin)
            ax_map.set_aspect("equal", adjustable="box")

        if inters:
            xi = [p[0] for p in inters]
            yi = [p[1] for p in inters]
            ax_map.scatter(xi, yi, s=80, marker="x", label="interseccion/apertura")

        if pares:
            xp = [p[0] for p in pares]
            yp = [p[1] for p in pares]
            ax_map.scatter(xp, yp, s=120, marker="s", label="PARE")

        ax_map.legend(loc="best")

        ax_scan.set_title(
            f"LiDAR /scan | F={snap['front']:.2f} L={snap['left']:.2f} R={snap['right']:.2f}"
        )
        ax_scan.set_theta_zero_location("N")
        ax_scan.set_theta_direction(-1)
        ax_scan.set_rlim(0, 3.0)
        ax_scan.grid(True)

        if snap["scan_angles"] and snap["scan_ranges"]:
            ax_scan.scatter(snap["scan_angles"], snap["scan_ranges"], s=5)

        fig.tight_layout()

        # Se guarda PNG cada 2 segundos, por si la ventana no se ve bien.
        now = time.time()
        if now - last_save > 2.0:
            png = os.path.join(node.outdir, "live_plot_final2.png")
            fig.savefig(png, dpi=120)
            last_save = now

        plt.pause(0.25)

    # Guardado final
    fig.savefig(os.path.join(node.outdir, "live_plot_final2_final.png"), dpi=150)
    fig.savefig(os.path.join(node.outdir, "live_plot_final2_final.svg"))


def main():
    rclpy.init()
    node = LiveData()

    thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    thread.start()

    try:
        run_plot(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
