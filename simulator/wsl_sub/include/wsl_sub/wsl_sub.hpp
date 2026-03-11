#ifndef LINE_DETECT_NODE_HPP
#define LINE_DETECT_NODE_HPP

#include <rclcpp/rclcpp.hpp>                     // Node 상속
#include <sensor_msgs/msg/compressed_image.hpp>  // 콜백 파라미터 타입
#include <opencv2/opencv.hpp>                    

// LineDetectNode 클래스
class LineDetectNode : public rclcpp::Node
{
public:
    LineDetectNode();  // 생성자

private:
    // 토픽 구독 Subscriber
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub;

    int width, height;
    cv::Point p_center;   // line 추적용 중심점
    // callback
    void line_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
};

// 함수
void Set(cv::Mat& frame);   //roi 설정
int Findline(cv::Mat& frame, cv::Point& p_center, cv::Mat& stats, cv::Mat& centroids); // 라인추적함수
void Draw(cv::Mat& frame,cv::Mat stats,cv::Mat centroids,int labels,int index,cv::Point p_center); // 시각화함수
#endif // LINE_DETECT_NODE_HPP
