#!/usr/bin/env python3
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

import rclpy
from rclpy.node import Node
import signal
import sys

Gst.init(None)


class FeedPreview(Node):
    def __init__(self):
        super().__init__("feed_preview")
        
        self.declare_parameter("port", 5001)
        self.port = self.get_parameter("port").value
        
        self.get_logger().info(f"Starting GStreamer preview on UDP port {self.port}")
        
        # Build the GStreamer pipeline
        pipeline_str = (
            f"udpsrc port={self.port} "
            f'caps="application/x-rtp,encoding-name=H264,payload=96" ! '
            f"rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false"
        )
        
        self.get_logger().info(f"Pipeline: {pipeline_str}")
        
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
        except GLib.Error as e:
            self.get_logger().error(f"Failed to create pipeline: {e}")
            raise
        
        # Set up bus to handle messages
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.on_message)
        
        # Start the pipeline
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error("Unable to set the pipeline to PLAYING state")
            raise RuntimeError("Pipeline failed to start")
        
        self.get_logger().info("Pipeline started successfully")
    
    def on_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.EOS:
            self.get_logger().info("End-of-stream")
            self.shutdown()
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            self.get_logger().error(f"Error: {err}, {debug}")
            self.shutdown()
        elif t == Gst.MessageType.WARNING:
            warn, debug = message.parse_warning()
            self.get_logger().warn(f"Warning: {warn}, {debug}")
        elif t == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old_state, new_state, pending_state = message.parse_state_changed()
                self.get_logger().info(
                    f"Pipeline state changed from {old_state.value_nick} "
                    f"to {new_state.value_nick}"
                )


def main():
    try:
        rclpy.init()
        node = FeedPreview()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
