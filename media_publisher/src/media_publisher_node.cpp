#include "media_publisher/media_publisher_node.hpp"

MediaPublisher::MediaPublisher(rclcpp::Node::SharedPtr node)
: node_(node), frame_index_(0)
{
    image_transport::ImageTransport it(node_);
    media_pub_ = it.advertise("mini_pupper_lcd/image", 10);
}

bool MediaPublisher::send_image(const std::string &image_path)
{
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (image.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to read image: %s", image_path.c_str());
        return false;
    }

    cv::resize(image, image, cv::Size(320, 240));
    publish_frame(image);
    return true;
}

bool MediaPublisher::send_video(const std::string &video_path)
{
    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open video: %s", video_path.c_str());
        return false;
    }

    cv::Mat frame;
    while (rclcpp::ok() && cap.read(frame)) {
        cv::resize(frame, frame, cv::Size(320, 240));
        publish_frame(frame);

        rclcpp::sleep_for(std::chrono::milliseconds(50));  // ~20 FPS
    }

    RCLCPP_INFO(node_->get_logger(), "Finished playing video: %s", video_path.c_str());
    return true;
}

void MediaPublisher::publish_frame(const cv::Mat &frame)
{
    std_msgs::msg::Header header;
    header.stamp = node_->get_clock()->now();
    auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

    media_pub_.publish(msg);
    RCLCPP_INFO(node_->get_logger(), "Published frame %d", frame_index_++);
}
