#include <rclcpp/rclcpp.hpp>
#include "media_publisher/media_publisher_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("media_sender_node");

    std::string media_path = "";

    if (argc > 1) media_path = argv[1];
    
    MediaPublisher publisher(node);

    // Send image or video based on extension
    if (media_path.ends_with(".jpg") || media_path.ends_with(".png")) {
        publisher.send_image(media_path);
    } else if (media_path.ends_with(".mp4") || media_path.ends_with(".gif")) {
        publisher.send_video(media_path);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Unsupported media type or wrong path: %s", media_path.c_str());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}

