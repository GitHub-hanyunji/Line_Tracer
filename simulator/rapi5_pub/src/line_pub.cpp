#include "jetson_pub/jetson_pub.hpp"
#include <cv_bridge/cv_bridge.h>
#include <chrono>

using namespace std::chrono_literals;


// 생성자
// Node 클래스 상속
VideoPubNode::VideoPubNode() : Node("video_publisher")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));   // qos
    // 퍼블리셔 생성
    pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed_13",qos);
    // 비디오 경로
    video_path_ = "/home/linux/ros2_foxy_ws/src/jetson_pub/simulation/5_lt_cw_100rpm_out.mp4";

    // 비디오 열기
    cap_.open(video_path_);
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open video file: %s", video_path_.c_str());
    }
    // 초당 30번 실행
    timer_ = this->create_wall_timer(30ms, std::bind(&VideoPubNode::publish_frame, this));
    RCLCPP_INFO(this->get_logger(), "VideoPubNode started");
}

// 영상 프레임을 읽어서 토픽으로 퍼블리시
void VideoPubNode::publish_frame()
{
    cv::Mat frame;  
    cap_ >> frame;  // 영상읽기

    // 영상 비어 있으면
    if (frame.empty()) {
        RCLCPP_INFO(this->get_logger(), "Restarting video");

        cap_.release();  // 영상 재시작을 위해 cap을 다시 열어줌
        cap_.open(video_path_);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not reopen video file");
        }
        return;
    }

    std_msgs::msg::Header header;
    // OpenCV 이미지 => ROS 이미지 메시지로 변환하는 브리지 라이브러리
    // "bgr8" : frame의 컬러 포맷(BGR 채널 3개)
    //.toCompressedImageMsg(): JPEG로 압축된 CompressedImage 생성
    auto msg = cv_bridge::CvImage(header, "bgr8", frame).toCompressedImageMsg();

    // msg 퍼블리시(스마트 포인트)
    pub_->publish(*msg);
}
