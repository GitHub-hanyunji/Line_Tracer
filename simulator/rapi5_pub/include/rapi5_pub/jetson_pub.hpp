#ifndef VIDEO_PUB_HPP
#define VIDEO_PUB_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>

class VideoPubNode : public rclcpp::Node
{
public:
    VideoPubNode();

private:
    void publish_frame();

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    cv::VideoCapture cap_;
    std::string video_path_;
};

#endif // VIDEO_PUB_HPP
