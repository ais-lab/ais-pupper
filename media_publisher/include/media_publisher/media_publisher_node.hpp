#pragma once

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

class MediaPublisher {
public:
    MediaPublisher(rclcpp::Node::SharedPtr node);

    bool send_image(const std::string &image_path);     // Load and publish a single image
    bool send_video(const std::string &video_path);     // Load and publish all frames of a video or gif

private:
    rclcpp::Node::SharedPtr node_;
    image_transport::Publisher media_pub_;
    int frame_index_;

    void publish_frame(const cv::Mat &frame);
};
