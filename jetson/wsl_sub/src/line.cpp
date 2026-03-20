#include "line_2/line.hpp"
#include <chrono>                 
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#define STDIN_FILENO 0

using namespace cv;

int getch(void)
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

bool kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return true;
    }
    return false;
}




// 관심영역 설정 함수
void Set(Mat& frame){
    frame = frame(Rect(Point(0, frame.rows * 3 / 4), Point(frame.cols, frame.rows)));  // ROI 하단 1/4
    cvtColor(frame, frame, COLOR_BGR2GRAY);  // Gray스케일
    frame += Scalar(100) - mean(frame);  // 밝기보정
    // 이진화 (범위 조정=>150이 제일 적당하다고 판단)
    threshold(frame, frame, 150, 255, THRESH_BINARY);
}

// 라인 추적 함수
int Findline(Mat& frame, Point& p_center, Mat& stats, Mat& centroids) {
    Mat labels;
    // connectedComponentsWithStats로 라인영역 찾기 -> 배경 포함한 blob 개수
    //stats: 각 객체의 bounding box + area
    //centroids: 각 blob의 무게중심
    int cnt = connectedComponentsWithStats(frame, labels, stats, centroids);

    // 우리의 중심점과 가장 가까운 객체 찾기
    int min_index = -1;   // 최소 인덱스
    int min_dist = frame.cols; // 거리 최소값 저장

    // 0번은 배경이니까 제외하고 반복문
    for (int i = 1; i < cnt; i++) {
        int area = stats.at<int>(i, 4); // 객체 면적
        
        // area가 너무 작으면 노이즈 skip, 50 이상만 
        if (area > 50) {
            // 객체 중심점 계산
            int x = cvRound(centroids.at<double>(i, 0));
            int y = cvRound(centroids.at<double>(i, 1));
            // 우리의 중심점과 객체 중심의 거리 구하기
            int dist = norm(Point(x, y) - p_center);
            //150px 이내에 있고 지금까지 본 객체들 중 가장 가까우면  그것을 후보로 저장
            if (dist < min_dist && dist <= 150) 
            {
                min_dist = dist;
                min_index = i;
            }
        }
    }
    // 설정한 최소 거리 내에 객체가 있는 경우
    if (min_index != -1 && min_dist <= 150) 
    {   // 중심점 갱신
        p_center = Point(cvRound(centroids.at<double>(min_index, 0)), cvRound(centroids.at<double>(min_index, 1)));  
    }
    // 못 찾았으면 기존 중심점에 빨간 점 표시 -> 추적 실패
    else circle(frame, Point(p_center.x, p_center.y), 5, Scalar(0, 0, 255), -1);

    // 업데이트된 p_center로 다시 한번 가장 가까운 blob을 추적 index로 확정
    int idx = -1;
    double best = frame.cols;
    
    // p_center에서 가장 가까운 객체를 찾는 반복문
    for(int i = 1; i < stats.rows; i++)
    {
        int cx = centroids.at<double>(i, 0);
        int cy = centroids.at<double>(i, 1);
        double d = norm(Point(cx, cy) - p_center);  // 우리 중심점과 객체 중심점 거리 구하기
    
        if (d < best)
        {
            best = d;
            idx = i;
        }
    }
    
    // 어느 정도 거리 이내일 때만 index 인정
    if (best > 30)   // 30px 이상 멀면 tracking 실패 처리
        idx = -1;
    
    return idx;
}

// 라인 시각화 함수
void Draw(Mat& frame, Mat stats, Mat centroids,int labels, int index, Point p_center)
{
    // 이진 이미지를 컬러 이미지로 변환
    cvtColor(frame, frame, COLOR_GRAY2BGR);

    // index 유효성 체크
    // labels->stats.rows
    // vaild=true-> 추적, vaild=false->추적x
    bool valid = (index >= 1 && index < labels);

    // 배경 건너 뜀
    for (int i = 1; i < labels; i++)
    {
        // 면적
        int area = stats.at<int>(i, 4);
        // 너무 작거나 큰 노이즈 skip
        if (area < 50 || area > 5000)
            continue;

        // 빨강: 내가 주행하는 라인, 파랑=내가 주행하지 않는 라인
        Scalar color = (valid && i == index) ? Scalar(0,0,255) : Scalar(255,0,0); // 파랑 = 아웃라인

        // 바운딩 박스
        rectangle(frame,Rect(stats.at<int>(i,0), stats.at<int>(i,1),stats.at<int>(i,2), stats.at<int>(i,3)),color, 2);
        // 중심점    
        circle(frame,Point(centroids.at<double>(i,0), centroids.at<double>(i,1)),3, color, -1);
    }

    // 객체 못찾았을때
    if (!valid)
    {
        return;
        // 라인 다시 찾았을 때 중심점 기준으로 다시 움직이기 위해서 빨간색으로 표시 => 중심점 주변 빨간 네모 표시
        rectangle(frame,Rect(p_center.x - 2, p_center.y - 2, 4, 4),Scalar(0,0,255), 2);
    }
}


// LineDetectNode 클래스
// 생성자
// rclcpp::Node를 상속한 클래스
LineDetectNode::LineDetectNode() : Node("linedetect_wsl")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));   // qos
    auto fn=std::bind(&LineDetectNode::line_callback, this, std::placeholders::_1);
    sub = this->create_subscription<sensor_msgs::msg::CompressedImage>("/image/compressed_13",qos,fn);
    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub",qos);

    width = 640;
    height = 360;
    // ROI 중앙
    p_center = cv::Point(width / 2, (height / 4) / 2);
    vel_msg.x = 0.0;
    vel_msg.y = 0.0;
    vel_msg.z = 0.0; // 필요 없으면 0

    RCLCPP_INFO(this->get_logger(), "Line Detect Node Started");
}

// callback 함수
void LineDetectNode::line_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 타임 스탬프
    // 프레임 처리 속도 측정하기 위한 시작 타임스탬프
    auto start = std::chrono::steady_clock::now();  
    

    // 압축된 이미지 메시지를 Mat으로 변환
    cv::Mat frame = cv::imdecode(msg->data, cv::IMREAD_COLOR);
    if(frame.empty()) return;

    cv::Mat roi = frame.clone();
    Set(roi);  // roi 설정

    cv::Mat labels, stats, centroids;
    // 라인 추적
    int idx=Findline(roi, p_center, stats, centroids);  

    // 시각화
    Draw(roi, stats, centroids, stats.rows, idx, p_center);

    int center_x = width / 2;
    // 위치오차 계산 ->  error: 라인에서 로봇이 얼마나 떨어져있는지
    // 화면 중앙- 현재 라인의 무게중심 X 좌표
    // error < 0 =>왼쪽으로 치우침, error > 0 => 오른쪽으로 치우침
    int error = center_x - p_center.x;
    
    

    // 좌/우 속도계산
    // 양족 속도명령을 동일하게
    float leftvel = 100 - k* error;
    float rightvel = -(100 + k* error);
    
    char ch;
    if(kbhit()) // 없으면 제어 멈춤
    {
        ch = getch();
        // 주행 중지, 동영상 저장 완료
        if(ch == 'q'){
            mode=false;
            writer.release();
        }
        // 주행 시작하고 동영상 저장 시작
        else if(ch == 's'){
            mode=true;
            int fourcc = cv::VideoWriter::fourcc('m','p','4','v');
            double fps = 30.0;  // 원하는 FPS로 설정해도 됨
            writer.open("line_tracer.mp4", fourcc, fps,
                        cv::Size(frame.cols, frame.rows), true);
            if (!writer.isOpened()) {
                printf("VideoWriter 열기 실패\n");
            } else {
                printf("VideoWriter 초기화 완료! 영상을 저장합니다.\n");
            }
        }
    }
    // 주행 중일때 속도값 갱신
    if(mode){
        vel_msg.x = leftvel;
        vel_msg.y = rightvel;
        vel_msg.z = 0.0; // 필요 없으면 0
        writer.write(frame);
    }
    // 주행 중지 중일때 속도값 0 
    else if(!mode){
        vel_msg.x = 0.0;
        vel_msg.y = 0.0;
        vel_msg.z = 0.0; // 필요 없으면 0
    }
    pub_->publish(vel_msg);  // 속도값 publish
    
    cv::imshow("frame", frame);  // 원본
    cv::imshow("Track", roi);    // ROI
    cv::waitKey(1);

    //프레임 처리 시간을 ms 단위로 변환
    auto end = std::chrono::steady_clock::now();
    float t = std::chrono::duration<float,std::milli>(end - start).count();
    RCLCPP_INFO(this->get_logger(), "err:%d lvel:%f rvel:%f time:%f", error,vel_msg.x,vel_msg.y, t);

}

