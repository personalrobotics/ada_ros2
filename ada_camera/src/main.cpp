#include <chrono>
#include <exception>
#include <opencv2/opencv.hpp>

// Include RealSense Cross Platform API
#include <librealsense2-net/rs_net.hpp>
#include <librealsense2/rs.hpp>

// ROS
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std::chrono_literals;

// Convert rs2::frame to cv::Mat
static cv::Mat frame_to_mat(const rs2::frame& f)
{
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r_rgb = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        Mat r_bgr;
        cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
        return r_bgr;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}

/* Camera Node */
class CameraNode : public rclcpp::Node
{
public:
  CameraNode() : Node("ada_camera")
  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};

    // Host Parameter
    param_desc.name = "host";
    param_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
    param_desc.description = "IP or Hostname of Camera";
    param_desc.read_only = true;
    declare_parameter("host", "192.168.2.7", param_desc);
  }

  void init_net_camera() {
    // Get Parameters
    auto host = get_parameter("host").get_parameter_value().get<std::string>();

    // Conect to network camera
    mDevice = std::make_shared<rs2::net_device>(host);
    mDevice->add_to(mContext);

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    mPipeline = rs2::pipeline(mContext);

    // Configure pipeline
    rs2::config cfg;
    //cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);

    // Start Pipeline
    mPipeline.start(cfg);
  }

  bool init(image_transport::ImageTransport & transport)
  try {
    // Setup ROS Publishers
    mColorPub = transport.advertiseCamera("~/color", 1);
    //mAlignedDepthPub = transport.advertiseCamera("~/aligned_depth", 1);

    // Init Camera
    init_net_camera();

    // Setup Timer
    mTimer = this->create_wall_timer(33ms, std::bind(&CameraNode::timer_callback, this));
    RCLCPP_INFO(get_logger(), "Initialization Successful");
    return true;
  } catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args()
              << "):\n    " << e.what() << std::endl;
    return false;
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), e.what());
    return false;
  }

private:
  // For reading and publishing camera frames
  void timer_callback() try
  {
    // Block program until frames arrive
    rs2::frameset frames = mPipeline.wait_for_frames(1000);

    auto color = frame_to_mat(frames.get_color_frame());

    std_msgs::msg::Header hdr;
    sensor_msgs::msg::CameraInfo::SharedPtr info = std::make_shared<sensor_msgs::msg::CameraInfo>();
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(hdr, "bgr8", color).toImageMsg();

    mColorPub.publish(msg, info);
  } catch (const rs2::error & e) {
    RCLCPP_WARN(get_logger(), "Did not receive frame, re-initializing.");
    init_net_camera();
  }

  // Camera
  rs2::context mContext;
  rs2::pipeline mPipeline;
  std::shared_ptr<rs2::net_device> mDevice;

  // ROS Objects
  image_transport::CameraPublisher mColorPub;
  image_transport::CameraPublisher mAlignedDepthPub;
  rclcpp::TimerBase::SharedPtr mTimer;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Init Camera Node
  auto node = std::make_shared<CameraNode>();
  image_transport::ImageTransport transport(node);
  if (!node->init(transport)) {
    rclcpp::shutdown();
    return -1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}