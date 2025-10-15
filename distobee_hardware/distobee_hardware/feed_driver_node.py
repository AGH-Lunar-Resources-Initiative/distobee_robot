#!/usr/bin/env python3
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

Gst.init(None)


def find_av_devices():
    monitor = Gst.DeviceMonitor()
    monitor.add_filter("Video/Source", None)
    monitor.start()
    
    devices = []
    for dev in monitor.get_devices():
        name = dev.get_display_name()
        props = dev.get_properties()
        if "AV TO USB2.0" in name and props.has_field("device.path"):
            devices.append(props.get_string("device.path"))
    
    monitor.stop()
    return devices


class FeedDriver(Node):
    def __init__(self):
        super().__init__("feed_driver")
        
        self.declare_parameter("udp_host", "192.168.1.20")
        self.declare_parameter("udp_port", 5000)
        self.host = self.get_parameter("udp_host").value
        self.port = self.get_parameter("udp_port").value
        
        self.devices = find_av_devices()
        self.pipeline = None
        self.current_device_idx = None
        
        if self.devices:
            for i, device in enumerate(self.devices):
                self.get_logger().info(f"Camera [{i}]: {device}")
        else:
            self.get_logger().warn("No AV TO USB2.0 devices found")
        
        self._switch_to(None)
        self.last_device_index = -1
        
        self.create_subscription(Int8, "set_feed", self.on_set_feed, 10)


    def _build_pipeline(self, device_path):
        # Build GStreamer pipeline: test pattern if device_path is None, camera feed otherwise
        if device_path is None:
            pipeline_str = (
                f"videotestsrc is-live=true ! "
                f"videoconvert ! x264enc tune=zerolatency speed-preset=superfast bitrate=1000 key-int-max=30 ! "
                f"rtph264pay pt=96 ! udpsink host={self.host} port={self.port}"
            )
        else:
            pipeline_str = (
                f"v4l2src device={device_path} ! "
                f"videoconvert ! videoscale ! videorate ! "
                f"video/x-raw,width=640,height=480,framerate=30/1 ! "
                f"x264enc tune=zerolatency speed-preset=superfast bitrate=2000 key-int-max=30 ! "
                f"rtph264pay pt=96 ! udpsink host={self.host} port={self.port}"
            )
        return Gst.parse_launch(pipeline_str)

    def _switch_to(self, device_path):
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        
        self.pipeline = self._build_pipeline(device_path)
        self.pipeline.set_state(Gst.State.PLAYING)

    def on_set_feed(self, msg: Int8):
        idx = msg.data
        
        if self.last_device_index == idx:
            return
        self.last_device_index = idx

        if idx == -1:
            self.current_device_idx = None
            self._switch_to(None)
            self.get_logger().info("Switched to test pattern")
            return
        
        if 0 <= idx < len(self.devices):
            self.current_device_idx = idx
            self._switch_to(self.devices[idx])
            self.get_logger().info(f"Switched to camera {idx}")
        else:
            self.get_logger().error(f"Invalid camera index: {idx}")

    def destroy_node(self):
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def main():
    try:
        rclpy.init()
        node = FeedDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
