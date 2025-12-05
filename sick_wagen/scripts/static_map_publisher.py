#!/usr/bin/env python3
import argparse
import math
from pathlib import Path
from typing import Tuple

import numpy as np
from PIL import Image
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import yaml

DEFAULT_MAP_YAML = "/home/sick/ros2_ws/maps/tsukuba.yaml"


def load_map(yaml_path: Path) -> Tuple[MapMetaData, list[int]]:
    with open(yaml_path, "r") as f:
        cfg = yaml.safe_load(f)

    image_path = Path(cfg["image"]).expanduser()
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path

    img = Image.open(image_path).convert("L")
    data = np.array(img, dtype=np.uint8)

    res = float(cfg.get("resolution", 0.05))
    ox, oy, oyaw = cfg.get("origin", [0.0, 0.0, 0.0])
    negate = int(cfg.get("negate", 0))
    occ_th = float(cfg.get("occupied_thresh", 0.65)) * 255
    free_th = float(cfg.get("free_thresh", 0.196)) * 255

    if negate:
        data = 255 - data
    # PGM origin is top-left; OccupancyGrid expects (0,0) at bottom-left.
    data = np.flipud(data)

    flat: list[int] = []
    for v in data.flatten():
        if v > occ_th:
            flat.append(0)  # free
        elif v < free_th:
            flat.append(100)  # occupied
        else:
            flat.append(-1)  # unknown

    meta = MapMetaData()
    meta.resolution = res
    meta.width = data.shape[1]
    meta.height = data.shape[0]
    meta.origin = Pose()
    meta.origin.position.x = ox
    meta.origin.position.y = oy
    meta.origin.orientation.z = math.sin(oyaw / 2.0)
    meta.origin.orientation.w = math.cos(oyaw / 2.0)
    return meta, flat


class StaticMapPublisher(Node):
    def __init__(self, yaml_path: Path) -> None:
        super().__init__("static_map_publisher")
        meta, data = load_map(yaml_path)

        self._msg = OccupancyGrid()
        self._msg.header.frame_id = "map"
        self._msg.info = meta
        self._msg.data = data

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(OccupancyGrid, "/map", qos)
        self.create_timer(1.0, self._publish)

        self.get_logger().info(f"Publishing static map from {yaml_path}")

    def _publish(self) -> None:
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._msg)


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish a static OccupancyGrid from a map YAML.")
    parser.add_argument("--map", dest="map_yaml", default=DEFAULT_MAP_YAML, help="Path to map YAML file.")
    args = parser.parse_args()

    rclpy.init()
    node = StaticMapPublisher(Path(args.map_yaml).expanduser())
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
