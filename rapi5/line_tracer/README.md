# Line Tracer
 
카메라 영상에서 단일 라인을 추적하고, 라인의 무게중심을 기준으로
position error를 계산하여 좌·우 바퀴 속도를 제어하는 라인 트레이서 패키지.
Raspberry Pi5에서 카메라 영상을 발행하고, WSL2에서 라인을 검출하여 속도 명령을 Dynamixel로 전달한다.
 
영상 수신 → ROI 설정 → 이진화 → 라인 검출 → 무게중심 갱신 → error 계산 → 속도 명령 발행
 
#### ▶ rapi5
https://github.com/GitHub-hanyunji/Line_Tracer/tree/main/rapi5/line_tracer/line_rapi5 <br>
camera_ros2 패키지와 dxl_nano 패키지 사용

#### ▶ wsl
아래 README.md 파일에 상세설명 <br>
https://github.com/GitHub-hanyunji/Line_Tracer/tree/main/rapi5/line_tracer/line_wsl
 
### \<결과영상\>
[![Line Tracer Robot](https://img.youtube.com/vi/MaP-0MkbB3g/0.jpg)](https://www.youtube.com/watch?v=MaP-0MkbB3g)
 
