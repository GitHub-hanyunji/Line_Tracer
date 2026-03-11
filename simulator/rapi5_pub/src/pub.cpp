#include "jetson_pub/jetson_pub.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); // ROS2 시스템 초기화
    // Node 실행 (spin)
    // 이 과정이 실행되는 동안 publish_frame() 타이머 함수가 계속 호출됨.
    rclcpp::spin(std::make_shared<VideoPubNode>());
    rclcpp::shutdown();  // ROS2 종료
    return 0;
}
