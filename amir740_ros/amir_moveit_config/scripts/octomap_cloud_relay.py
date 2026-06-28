#!/usr/bin/env python3
"""
MoveIt octomap 用の点群リレー (フレーム補正 + ON/OFF ゲート)

目的:
  - gz の /d435/points は gz規約(X前方/Z上)のデータなのに frame_id が
    optical(Z前方)になっている不整合がある。MoveIt の OccupancyMapMonitor は
    cloud の frame_id で TF を引くため、そのまま渡すと octomap が 90°倒れる。
    → header.frame_id を d435_depth_frame(X前方/Z上のボディ規約) に書き換えて再配信。
  - ON/OFF ゲートも兼ねる。enabled=false の間は再配信を止めるので、MoveIt に
    新しい点群が届かず octomap が更新されない(="topicが来たら表示"と同じ挙動)。

ON/OFF 操作 (実行中に切替):
  ros2 topic pub -1 /octomap_enable std_msgs/Bool "{data: true}"   # ON
  ros2 topic pub -1 /octomap_enable std_msgs/Bool "{data: false}"  # OFF
  ※OFF にしても既存の octomap は残る。消すには:
  ros2 service call /clear_octomap std_srvs/srv/Empty

パラメータ:
  input_topic   (string, 既定 /d435/points)        : 入力点群
  output_topic  (string, 既定 /moveit_cloud)        : MoveIt に渡す点群
  override_frame(string, 既定 d435_depth_frame)     : 書き換える frame_id (空なら無変更)
  enabled       (bool,   既定 True)                 : 起動時の ON/OFF
  enable_topic  (string, 既定 /octomap_enable)      : ON/OFF 切替トピック
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


class OctomapCloudRelay(Node):
    def __init__(self):
        super().__init__('octomap_cloud_relay')
        self.input_topic = self.declare_parameter('input_topic', '/d435/points').value
        self.output_topic = self.declare_parameter('output_topic', '/moveit_cloud').value
        self.override_frame = self.declare_parameter('override_frame', 'd435_depth_frame').value
        self.enabled = self.declare_parameter('enabled', True).value
        enable_topic = self.declare_parameter('enable_topic', '/octomap_enable').value

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, qos)
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self._cb, qos)
        # ON/OFF (latched 風: transient_local で後から pub しても受かる)
        enable_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                history=HistoryPolicy.KEEP_LAST)
        self.enable_sub = self.create_subscription(Bool, enable_topic, self._enable_cb, enable_qos)

        self.get_logger().info(
            f"octomap_cloud_relay: {self.input_topic} -> {self.output_topic} "
            f"(frame='{self.override_frame or '(無変更)'}', enabled={self.enabled}, "
            f"toggle via {enable_topic})")

    def _enable_cb(self, msg: Bool):
        if msg.data != self.enabled:
            self.get_logger().info(f"octomap relay {'ON' if msg.data else 'OFF'}")
        self.enabled = msg.data

    def _cb(self, msg: PointCloud2):
        if not self.enabled:
            return
        if self.override_frame:
            msg.header.frame_id = self.override_frame
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(OctomapCloudRelay())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
