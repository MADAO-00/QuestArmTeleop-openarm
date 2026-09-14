#!/usr/bin/env python3
"""Quest pose publisher for the fail-closed OpenArm v1 ROS control path."""

from __future__ import annotations

from math import isfinite
import socket
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, UInt64
from tf2_ros import TransformBroadcaster

from oculus_reader import OculusReader
from openarm_teleop_core import (
    is_rclpy_humble_shutdown_take_race,
    update_grip_deadman,
)
from transformations import quaternion_from_matrix


BUTTON_NAMES = ("LG", "RG", "A", "B", "X", "Y", "LJ", "RJ")
AXIS_NAMES = ("leftTrig", "rightTrig", "leftGrip", "rightGrip")
GRIP_CHANNELS = {
    "left": ("LG", "leftGrip"),
    "right": ("RG", "rightGrip"),
}


class TimestampedOculusReader(OculusReader):
    """Add an atomic source timestamp without modifying the Piper reader."""

    STOP_JOIN_TIMEOUT_SEC = 0.5

    def __init__(self, *args, **kwargs):
        self._snapshot_received_ns = 0
        self._snapshot_sequence = 0
        # OculusReader.__init__ may call run(), so the stream state must exist
        # before entering the base constructor.
        self._stream_lock = threading.Lock()
        self._stream_connection = None
        self._stream_file = None
        super().__init__(*args, **kwargs)

    def run(self):
        """Start logcat on a daemon thread owned by this OpenArm reader."""

        self.running = True
        self.device.shell(
            'am start -n "com.rail.oculus.teleop/'
            'com.rail.oculus.teleop.MainActivity" '
            "-a android.intent.action.MAIN "
            "-c android.intent.category.LAUNCHER"
        )
        self.thread = threading.Thread(
            target=self.device.shell,
            args=("logcat -T 0", self.read_logcat_by_line),
            name="openarm-quest-logcat",
            daemon=True,
        )
        self.thread.start()

    @staticmethod
    def _close_stream_connection(connection) -> None:
        """Wake a blocking socket readline and close its ADB connection."""

        raw_socket = getattr(connection, "socket", None)
        if raw_socket is not None:
            try:
                raw_socket.shutdown(socket.SHUT_RDWR)
            except (AttributeError, OSError, RuntimeError):
                pass
        try:
            connection.close()
        except (AttributeError, OSError, RuntimeError):
            pass

    def stop(self):
        """Stop without allowing a stuck ADB readline to block ROS shutdown."""

        self.running = False
        with self._stream_lock:
            connection = self._stream_connection
        if connection is not None:
            self._close_stream_connection(connection)

        thread = getattr(self, "thread", None)
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=self.STOP_JOIN_TIMEOUT_SEC)

    def get_snapshot(self):
        """Return one internally consistent packet snapshot."""

        with self._lock:
            transforms = {
                side: value.copy() for side, value in self.last_transforms.items()
            }
            buttons = {
                key: list(value) if isinstance(value, (list, tuple)) else value
                for key, value in self.last_buttons.items()
            }
            return (
                transforms,
                buttons,
                int(self._snapshot_received_ns),
                int(self._snapshot_sequence),
            )

    def read_logcat_by_line(self, connection):
        """Store timestamp, buttons, and both transforms in one critical section."""

        file_obj = None
        with self._stream_lock:
            self._stream_connection = connection
        try:
            if not self.running:
                return
            file_obj = connection.socket.makefile()
            with self._stream_lock:
                self._stream_file = file_obj
            while self.running:
                try:
                    line = file_obj.readline()
                    if not line:
                        break
                    line = line.strip()
                    data = self.extract_data(line)
                    if not data:
                        continue
                    transforms, buttons = OculusReader.process_data(data)
                    if (
                        not isinstance(transforms, dict)
                        or not isinstance(buttons, dict)
                        or transforms.get("r") is None
                        or transforms.get("l") is None
                    ):
                        continue
                    received_ns = time.monotonic_ns()
                    with self._lock:
                        self.last_transforms = transforms
                        self.last_buttons = buttons
                        self._snapshot_received_ns = received_ns
                        self._snapshot_sequence += 1
                    if self.print_FPS:
                        self.fps_counter.getAndPrintFPS()
                except UnicodeDecodeError:
                    continue
                except (OSError, ValueError):
                    if not self.running:
                        break
                    raise
        except (OSError, ValueError):
            # stop() may close the ADB socket after the running check but
            # before socket.makefile() completes.  That is an expected wakeup,
            # not a reader-thread failure worth printing during ROS shutdown.
            if self.running:
                raise
        finally:
            with self._stream_lock:
                if self._stream_file is file_obj:
                    self._stream_file = None
                if self._stream_connection is connection:
                    self._stream_connection = None
            if file_obj is not None:
                try:
                    file_obj.close()
                except (OSError, ValueError):
                    pass
            self._close_stream_connection(connection)


class OpenArmOculusPublisher(Node):
    """Publish synchronized Quest packets and explicitly signal source validity."""

    def __init__(self):
        super().__init__("pub_pose_openarm_v1_node")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("parent_frame_id", "arm_origin")
        self.declare_parameter("button_state_topic", "/oculus/buttons")
        self.declare_parameter("source_valid_topic", "/oculus/source_valid")
        self.declare_parameter("source_epoch_topic", "/oculus/source_epoch")
        self.declare_parameter("right_handle_pose_topic", "/right_handle_pose")
        self.declare_parameter("left_handle_pose_topic", "/left_handle_pose")
        self.declare_parameter("source_timeout_sec", 0.25)
        self.declare_parameter("grip_press_threshold", 0.55)
        self.declare_parameter("grip_release_threshold", 0.35)
        self.parent_frame_id = str(self.get_parameter("parent_frame_id").value)
        self.source_timeout_sec = float(self.get_parameter("source_timeout_sec").value)
        if self.source_timeout_sec <= 0.0:
            raise ValueError("source_timeout_sec must be positive")
        self.grip_press_threshold = float(
            self.get_parameter("grip_press_threshold").value
        )
        self.grip_release_threshold = float(
            self.get_parameter("grip_release_threshold").value
        )
        # Validate once at startup through the shared pure helper.
        update_grip_deadman(
            raw_pressed=False,
            analog_value=0.0,
            previous=False,
            press_threshold=self.grip_press_threshold,
            release_threshold=self.grip_release_threshold,
        )

        self.right_pose_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("right_handle_pose_topic").value), 1
        )
        self.left_pose_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("left_handle_pose_topic").value), 1
        )
        self.button_pub = self.create_publisher(
            Joy, str(self.get_parameter("button_state_topic").value), 1
        )
        self.source_valid_pub = self.create_publisher(
            Bool, str(self.get_parameter("source_valid_topic").value), 1
        )
        self.source_epoch_pub = self.create_publisher(
            UInt64, str(self.get_parameter("source_epoch_topic").value), 1
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self._openxr_to_ros = np.array(
            [
                [0.0, 0.0, -1.0, 0.0],
                [-1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        self._ros_to_openxr = np.linalg.inv(self._openxr_to_ros)

        self.reader = TimestampedOculusReader()
        self._last_sequence = 0
        self._source_was_valid = False
        self._grip_active = {"left": False, "right": False}

        publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._timer_callback)
        self.get_logger().info(
            f"OpenArm Quest publisher ready; source timeout={self.source_timeout_sec:.3f}s"
        )

    def _correct_to_arm(self, transform: np.ndarray) -> np.ndarray:
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            raise ValueError("Quest transform must be a finite 4x4 matrix")
        if not np.allclose(
            transform[3], np.array((0.0, 0.0, 0.0, 1.0)), atol=1e-9
        ):
            raise ValueError("Quest transform must be homogeneous")
        return self._openxr_to_ros @ transform @ self._ros_to_openxr

    @staticmethod
    def _axis_value(buttons: dict, name: str) -> float:
        value = buttons.get(name, [0.0])
        if isinstance(value, (list, tuple)):
            value = value[0] if value else 0.0
        if not isinstance(value, (int, float)):
            return 0.0
        result = float(value)
        return max(0.0, min(result, 1.0)) if isfinite(result) else 0.0

    def _publish_buttons(self, buttons: dict, stamp) -> None:
        msg = Joy()
        msg.header.stamp = stamp
        msg.header.frame_id = self.parent_frame_id
        axis_values = {name: self._axis_value(buttons, name) for name in AXIS_NAMES}
        synthesized = {name: bool(buttons.get(name, False)) for name in BUTTON_NAMES}
        for side, (button_name, axis_name) in GRIP_CHANNELS.items():
            previous = self._grip_active[side]
            active = update_grip_deadman(
                raw_pressed=bool(buttons.get(button_name, False)),
                analog_value=axis_values[axis_name],
                previous=previous,
                press_threshold=self.grip_press_threshold,
                release_threshold=self.grip_release_threshold,
            )
            self._grip_active[side] = active
            synthesized[button_name] = active
            if active != previous:
                self.get_logger().info(
                    f"Quest {side} Grip {'engaged' if active else 'released'}: "
                    f"raw={bool(buttons.get(button_name, False))}, "
                    f"analog={axis_values[axis_name]:.3f}"
                )
        msg.axes = [axis_values[name] for name in AXIS_NAMES]
        msg.buttons = [1 if synthesized[name] else 0 for name in BUTTON_NAMES]
        self.button_pub.publish(msg)

    def _publish_pose(self, transform: np.ndarray, publisher, child_frame: str, stamp) -> None:
        quat = quaternion_from_matrix(transform)
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.parent_frame_id
        pose.pose.position.x = float(transform[0, 3])
        pose.pose.position.y = float(transform[1, 3])
        pose.pose.position.z = float(transform[2, 3])
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])
        publisher.publish(pose)

        transform_msg = TransformStamped()
        transform_msg.header = pose.header
        transform_msg.child_frame_id = child_frame
        transform_msg.transform.translation.x = pose.pose.position.x
        transform_msg.transform.translation.y = pose.pose.position.y
        transform_msg.transform.translation.z = pose.pose.position.z
        transform_msg.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform_msg)

    def _publish_invalid(self, stamp) -> None:
        self.source_valid_pub.publish(Bool(data=False))
        self._publish_buttons({}, stamp)

    def _timer_callback(self) -> None:
        transforms, buttons, received_ns, sequence = self.reader.get_snapshot()
        now_ns = time.monotonic_ns()
        stamp = self.get_clock().now().to_msg()
        source_valid = (
            received_ns > 0
            and 0 <= now_ns - received_ns <= int(self.source_timeout_sec * 1e9)
        )

        if not source_valid:
            self._publish_invalid(stamp)
            if self._source_was_valid:
                self.get_logger().error("Quest source stale; neutralizing controls")
            self._source_was_valid = False
            return

        if sequence == self._last_sequence:
            return

        try:
            right = self._correct_to_arm(transforms["r"])
            left = self._correct_to_arm(transforms["l"])
        except (KeyError, TypeError, ValueError) as exc:
            self._publish_invalid(stamp)
            self._source_was_valid = False
            self.get_logger().error(f"Rejecting invalid Quest packet: {exc}")
            return

        self._publish_pose(right, self.right_pose_pub, "right_controller", stamp)
        self._publish_pose(left, self.left_pose_pub, "left_controller", stamp)
        self._publish_buttons(buttons, stamp)
        self.source_epoch_pub.publish(UInt64(data=received_ns))
        self.source_valid_pub.publish(Bool(data=True))
        self._last_sequence = sequence
        self._source_was_valid = True

    def destroy_node(self):
        self.reader.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OpenArmOculusPublisher()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except RuntimeError as exc:
            shutdown_take_race = is_rclpy_humble_shutdown_take_race(
                exc, context_ok=rclpy.ok()
            )
            if not shutdown_take_race:
                raise
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
