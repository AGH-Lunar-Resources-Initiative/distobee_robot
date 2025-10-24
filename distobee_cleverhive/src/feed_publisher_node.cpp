#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <opencv2/opencv.hpp>

namespace distobee_cleverhive {

class FeedPublisher : public rclcpp::Node
{
public:
  FeedPublisher(const rclcpp::NodeOptions &options) : Node("feed_publisher", options)
  {
    this->declare_parameter<int>("port", 5001);
    this->get_parameter("port", port_);

    RCLCPP_INFO(this->get_logger(), "Starting GStreamer feed publisher on UDP port %d", port_);

    // Initialize GStreamer
    gst_init(nullptr, nullptr);

    // Create publisher directly without image_transport for now
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 1);

    // Build the GStreamer pipeline
    std::string pipeline_str = 
        "udpsrc port=" + std::to_string(port_) + " "
        "caps=\"application/x-rtp,encoding-name=H264,payload=96\" ! "
        "rtph264depay ! avdec_h264 ! videoconvert ! "
        "video/x-raw,format=BGR ! appsink name=sink sync=false";

    RCLCPP_INFO(this->get_logger(), "Pipeline: %s", pipeline_str.c_str());

    GError *error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
    
    if (error != nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create pipeline: %s", error->message);
      g_error_free(error);
      throw std::runtime_error("Failed to create GStreamer pipeline");
    }

    // Get the appsink element
    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
    if (!appsink_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get appsink element");
      throw std::runtime_error("Failed to get appsink element");
    }

    // Set appsink properties
    g_object_set(G_OBJECT(appsink_), "emit-signals", TRUE, nullptr);
    g_signal_connect(appsink_, "new-sample", G_CALLBACK(on_new_sample_static), this);

    // Set up bus to handle messages
    GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));
    gst_bus_add_watch(bus, bus_call_static, this);
    gst_object_unref(bus);

    // Start the pipeline
    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(this->get_logger(), "Unable to set the pipeline to PLAYING state");
      throw std::runtime_error("Pipeline failed to start");
    }

    RCLCPP_INFO(this->get_logger(), "Pipeline started successfully");

    // Create a timer to process GStreamer events
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        [this]() { g_main_context_iteration(nullptr, FALSE); });
  }

  ~FeedPublisher()
  {
    if (pipeline_) {
      RCLCPP_INFO(this->get_logger(), "Stopping pipeline");
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);
    }
    if (appsink_) {
      gst_object_unref(appsink_);
    }
  }

private:
  static GstFlowReturn on_new_sample_static(GstElement *sink, gpointer user_data)
  {
    FeedPublisher *self = static_cast<FeedPublisher *>(user_data);
    return self->on_new_sample(sink);
  }

  GstFlowReturn on_new_sample(GstElement *sink)
  {
    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (!sample) {
      return GST_FLOW_ERROR;
    }

    GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *structure = gst_caps_get_structure(caps, 0);
    
    int width, height;
    gst_structure_get_int(structure, "width", &width);
    gst_structure_get_int(structure, "height", &height);

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);

    // Create OpenCV Mat from the buffer
    cv::Mat frame(height, width, CV_8UC3, (void *)map.data);

    // Convert to ROS2 message
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera";

    sensor_msgs::msg::Image::SharedPtr msg = 
        cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

    // Publish the message
    image_pub_->publish(*msg);

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    return GST_FLOW_OK;
  }

  static gboolean bus_call_static(GstBus *bus, GstMessage *msg, gpointer user_data)
  {
    FeedPublisher *self = static_cast<FeedPublisher *>(user_data);
    return self->bus_call(bus, msg);
  }

  gboolean bus_call(GstBus *bus, GstMessage *msg)
  {
    (void)bus;
    
    switch (GST_MESSAGE_TYPE(msg)) {
      case GST_MESSAGE_EOS:
        RCLCPP_INFO(this->get_logger(), "End-of-stream");
        break;
      case GST_MESSAGE_ERROR: {
        GError *err;
        gchar *debug;
        gst_message_parse_error(msg, &err, &debug);
        RCLCPP_ERROR(this->get_logger(), "Error: %s", err->message);
        g_error_free(err);
        g_free(debug);
        break;
      }
      case GST_MESSAGE_WARNING: {
        GError *warn;
        gchar *debug;
        gst_message_parse_warning(msg, &warn, &debug);
        RCLCPP_WARN(this->get_logger(), "Warning: %s", warn->message);
        g_error_free(warn);
        g_free(debug);
        break;
      }
      case GST_MESSAGE_STATE_CHANGED: {
        if (GST_MESSAGE_SRC(msg) == GST_OBJECT(pipeline_)) {
          GstState old_state, new_state, pending_state;
          gst_message_parse_state_changed(msg, &old_state, &new_state, &pending_state);
          RCLCPP_INFO(this->get_logger(), 
                      "Pipeline state changed from %s to %s",
                      gst_element_state_get_name(old_state),
                      gst_element_state_get_name(new_state));
        }
        break;
      }
      default:
        break;
    }

    return TRUE;
  }

  int port_;
  GstElement *pipeline_{nullptr};
  GstElement *appsink_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace distobee_cleverhive

RCLCPP_COMPONENTS_REGISTER_NODE(distobee_cleverhive::FeedPublisher)
