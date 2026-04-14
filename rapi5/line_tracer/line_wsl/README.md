# Wsl
## Line Tracer

### 작성자: 2301510 한윤지

카메라 영상(`CompressedImage`)을 구독하여 단일 라인을 추적하고,
라인의 무게중심을 기준으로 position error를 계산하여
좌·우 바퀴 속도를 `topic_dxlpub`으로 발행하는 Line Tracer 노드이다.

영상 수신 → ROI 설정 → 이진화 → 라인 검출 → 무게중심 갱신 → error 계산 → 속도 발행

---

## 파일 구성

| 파일 | 설명 |
|---|---|
| `line.hpp` | 클래스 선언, 멤버 변수 및 전역 함수 선언 |
| `line.cpp` | 각 함수 구현부 |
| `main.cpp` | ROS2 초기화 및 노드 실행 |

---



## 클래스 구조 (`line.hpp`)

`rclcpp::Node`를 상속하는 `LineDetectNode` 클래스이다.
`/image/compressed_13` 토픽을 구독하고 `topic_dxlpub` 토픽을 발행한다.

```cpp
#ifndef LINE_DETECT_NODE_HPP
#define LINE_DETECT_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <opencv2/opencv.hpp>

class LineDetectNode : public rclcpp::Node
{
public:
    LineDetectNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_;

    int width, height;          // frame 크기 (640×480)
    cv::Point p_center;         // 라인 추적용 무게중심점
    geometry_msgs::msg::Vector3 vel_msg;  // 발행할 속도 메시지
    bool mode = false;          // 주행 모드 플래그
    double k = 0.1;             // 비례 제어 이득
    cv::VideoWriter writer;     // mp4 저장
    bool video_key = false;     // 녹화 ON/OFF 플래그
    int last_error = 0;         // 이전 프레임 error 저장
    bool line_lost = false;     // 라인 소실 여부
    int lost_count = 0;         // 라인 소실 카운트

    void line_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
};

// 전역 함수 선언
int  getch(void);
bool kbhit(void);
void Set(cv::Mat& frame);
int  Findline(cv::Mat& frame, cv::Point& p_center, cv::Mat& stats, cv::Mat& centroids);
void Draw(cv::Mat& frame, cv::Mat stats, cv::Mat centroids,
          int labels, int index, cv::Point p_center);

#endif
```

---

## 함수 설명

#### 1. 생성자 `LineDetectNode()`
`/image/compressed_13` 토픽을 구독하고 `topic_dxlpub` 발행자를 초기화한다.
frame 크기(640×480)와 라인 추적 초기 중심점을 설정한다.

```cpp
LineDetectNode::LineDetectNode() : Node("linedetect_wsl")
{
    sub  = this->create_subscription<sensor_msgs::msg::CompressedImage>(
           "/image/compressed_13", qos, fn);
    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", qos);

    width = 640; height = 480;
    p_center = cv::Point(width / 2, (height / 4) / 2);  // ROI 중앙
    vel_msg.x = vel_msg.y = vel_msg.z = 0.0;
}
```

---

#### 2. `Set()` 함수
입력 영상의 하단 1/4만 잘라서 ROI로 만들고, 그레이스케일 변환 → 밝기 보정 →
이진화로 흰색 라인을 추출하는 전처리 함수이다.

```cpp
void Set(Mat& frame)
{
    // 하단 1/4 ROI 추출
    frame = frame(Rect(Point(0, frame.rows * 3 / 4), Point(frame.cols, frame.rows)));

    cvtColor(frame, frame, COLOR_BGR2GRAY);           // 그레이스케일 변환
    frame += Scalar(100) - mean(frame);               // 밝기 보정 (평균 기반)
    threshold(frame, frame, 120, 255, THRESH_BINARY); // 이진화
}
```

---

#### 3. `Findline()` 함수
ROI 이진 영상에서 연결된 컴포넌트를 분리한 뒤,
현재 `p_center`에서 70px 이내에서 가장 가까운 컴포넌트를 라인으로 선택한다.
선택된 라인의 무게중심으로 `p_center`를 갱신하고 인덱스를 반환한다.
라인을 찾지 못하면 기존 `p_center`에 빨간 점을 표시한다.

```cpp
// 1차 탐색: p_center 기준 70px 이내 가장 가까운 컴포넌트 선택
// p_center 갱신
// 2차 탐색: 갱신된 p_center 기준으로 50px 이내 최종 인덱스 확정
// 탐색 실패 시 idx = -1 반환
```

| 항목 | 값 |
|---|---|
| 1차 탐색 반경 | 70px |
| 2차 확정 반경 | 50px |
| 면적 필터 | 100px 이상 |

---

#### 4. `Draw()` 함수
검출된 컴포넌트에 바운딩 박스와 무게중심 점을 그린다.
추적 중인 라인은 빨간색, 나머지는 파란색으로 표시한다.

```cpp
// 추적 라인 (index 일치) → 빨간색 박스
// 나머지 컴포넌트       → 파란색 박스
// 라인 미검출 시        → p_center 주변 빨간 사각형
// 면적 필터: 100 ~ 7500px
```

---

#### 5. `line_callback()` 함수
`/image/compressed_13` 토픽으로 수신된 CompressedImage를 디코딩하여
Line Tracer 알고리즘을 수행하고 속도를 발행하는 핵심 제어 루프이다.

```cpp
void LineDetectNode::line_callback(...)
{
    // 1. CompressedImage 디코딩 → OpenCV Mat
    // 2. Set(roi)     → ROI + 이진화
    // 3. Findline()   → 라인 검출 + p_center 갱신
    // 4. Draw()       → 시각화
    // 5. error = (width / 2) - p_center.x
    // 6. leftvel  =   100 - k * error
    //    rightvel = -(100 + k * error)
    // 7. kbhit() 감지
    //      's' → mode=true + VideoWriter 초기화 (line_tracer.mp4, 30fps)
    //      'q' → mode=false + writer.release()
    // 8. mode=true  → vel_msg = (leftvel, rightvel, 0) + writer.write(frame)
    //    mode=false → vel_msg = (0, 0, 0)
    // 9. pub_->publish(vel_msg)
}
```

---

#### 6. `getch()` / `kbhit()` 함수
엔터 없이 키보드 입력을 즉시 감지하기 위한 터미널 제어 전역 함수이다.

```cpp
int  getch();   // 키 하나 즉시 읽기
bool kbhit();   // 키 입력 대기 여부 확인 (non-blocking)
```

---

## main.cpp

ROS2를 초기화하고 `LineDetectNode`를 실행한다.
`rclcpp::spin()`으로 무한 대기하며 `/image/compressed_13` 메시지가 들어올 때마다 콜백을 호출한다.

```cpp
#include "line_2/line.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);                               // ROS2 초기화
    rclcpp::spin(std::make_shared<LineDetectNode>());       // 노드 실행 (무한 대기)
    rclcpp::shutdown();                                     // ROS2 종료
}
```

---

## error 계산

```
error = (영상 중심 x) - (라인 무게중심 x)
      = width / 2 - p_center.x
```

| error | 의미 | 동작 |
|---|---|---|
| 0 | 라인이 정중앙 | 직진 |
| 양수 (+) | 라인이 왼쪽에 있음 | 우회전 |
| 음수 (-) | 라인이 오른쪽에 있음 | 좌회전 |

---

## 키보드 제어

| 키 | 동작 |
|---|---|
| `s` | 주행 시작 + 녹화 시작 (`line_tracer.mp4`, 30fps) |
| `q` | 주행 정지 + 녹화 저장 완료 |

---

## 저장 파일

| 파일명 | FPS | 해상도 | 내용 |
|---|---|---|---|
| `line_tracer.mp4` | 30fps | 640×480 | 원본 카메라 영상 |

---

## 토픽

| 방향 | 토픽명 | 타입 | 설명 |
|---|---|---|---|
| 구독 | `/image/compressed_13` | `sensor_msgs/CompressedImage` | 카메라 영상 |
| 발행 | `topic_dxlpub` | `geometry_msgs/Vector3` | 좌·우 바퀴 속도 (x=lvel, y=rvel) |
