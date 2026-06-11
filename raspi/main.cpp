#include <opencv2/opencv.hpp>
#include <iostream>
#include <fcntl.h>   // 시리얼 파일 제어 (O_RDWR, O_NOCTTY 등)
#include <termios.h> // POSIX 시리얼 통신 구조체 및 옵션
#include <unistd.h>  // read(), write(), close()

// STM32와의 시리얼 통신 설정을 위한 함수 (넌블로킹 모드)
int initSerial(const std::string& port, speed_t baudrate) {
    int serial_port = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port < 0) {
        std::cerr << "시리얼 포트를 열 수 없습니다: " << port << std::endl;
        return -1;
    }

    // 파일 제어 플래그를 읽기 시 대기하지 않도록(넌블로킹) 설정
    fcntl(serial_port, F_SETFL, FNDELAY);

    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "시리얼 속성을 가져오지 못했습니다." << std::endl;
        close(serial_port);
        return -1;
    }

    // 통신 속도 설정 (기본 115200)
    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    // 8N1 설정 (8비트 데이터, 패리티 없음, 1스톱 비트)
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CREAD | CLOCAL; 

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0; 

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "시리얼 속성을 설정하지 못했습니다." << std::endl;
        close(serial_port);
        return -1;
    }

    return serial_port;
}

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "카메라를 열 수 없습니다!" << std::endl;
        return -1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // [STM32 통신 초기화] 
    // USB to UART 모듈 연결 상태에 맞춰 포트명을 확인하세요. (기본 /dev/ttyUSB0)
    int serial_fd = initSerial("/dev/ttyUSB0", B115200);
    if (serial_fd >= 0) {
        std::cout << "STM32 시리얼 통신 연결 성공! 'S' 신호를 대기합니다." << std::endl;
    } else {
        std::cerr << "통신 포트를 열 수 없어 프로그램을 종료합니다." << std::endl;
        return -1;
    }

    cv::Mat frame, blurred, hsv, ycrcb;
    cv::Mat blue_mask, red_mask1, red_mask2, red_mask, total_normal_mask;

    // 루프 진입 전 안내
    std::cout << "라즈베리파이 가동 중... STM32 명령 대기 중 (종료: ESC)" << std::endl;

    while (true) {
        // 프리뷰 화면이 끊기지 않도록 프레임은 상시 받아옵니다.
        cap >> frame;
        if (frame.empty()) break;

        cv::Mat result = frame.clone();

        // STM32로부터 데이터가 1바이트 들어왔는지 체크
        char rx_buf = 0;
        int bytes_read = read(serial_fd, &rx_buf, 1);

        // STM32가 검사 시작 요청 신호('S')를 보낸 타이밍에만 하단 알고리즘 실행
        if (bytes_read > 0 && rx_buf == 'S') {
            std::cout << "\n[수신] STM32 요청('S') -> 비전 검사 프로세스 시작" << std::endl;

            // 1. CLAHE 조명 불균형 보정
            cv::cvtColor(frame, ycrcb, cv::COLOR_BGR2YCrCb);
            std::vector<cv::Mat> channels;
            cv::split(ycrcb, channels);

            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
            clahe->setClipLimit(3.0);       
            clahe->setTilesGridSize(cv::Size(8, 8)); 
            clahe->apply(channels[0], channels[0]);

            cv::merge(channels, ycrcb);
            cv::Mat equalized;
            cv::cvtColor(ycrcb, equalized, cv::COLOR_YCrCb2BGR);

            // 2. 가우시안 블러 및 HSV 변환
            cv::GaussianBlur(equalized, blurred, cv::Size(5, 5), 0);
            cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

            // 3. 확장된 범위의 파란색 마스크
            cv::Scalar lower_blue = cv::Scalar(95, 30, 20);   
            cv::Scalar upper_blue = cv::Scalar(135, 255, 255); 
            cv::inRange(hsv, lower_blue, upper_blue, blue_mask);

            // 4. 확장된 범위의 빨간색 마스크 (0근처, 180근처 병합)
            cv::inRange(hsv, cv::Scalar(0, 30, 20), cv::Scalar(12, 255, 255), red_mask1);
            cv::inRange(hsv, cv::Scalar(168, 30, 20), cv::Scalar(180, 255, 255), red_mask2);
            cv::bitwise_or(red_mask1, red_mask2, red_mask);

            // 5. 마스크 병합 및 모폴로지 노이즈 제거
            cv::bitwise_or(blue_mask, red_mask, total_normal_mask);
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
            cv::morphologyEx(total_normal_mask, total_normal_mask, cv::MORPH_CLOSE, kernel);
            cv::morphologyEx(total_normal_mask, total_normal_mask, cv::MORPH_OPEN, kernel);

            // 6. 윤곽선 추출 및 검증
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(total_normal_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            bool is_target_present = false;
            for (const auto& contour : contours) {
                double area = cv::contourArea(contour);
                
                // 설정하신 최적 면적 조건 (80000 ~ 300000)
                if (area > 80000 && area < 300000) { 
                    is_target_present = true;
                    cv::Rect rect = cv::boundingRect(contour);
                    cv::rectangle(result, rect, cv::Scalar(0, 255, 0), 2);
                }
            }

            // 7. 결과 확정 및 STM32 유선 전송
            char tx_data = 'N';
            if (is_target_present) {
                tx_data = 'O';
                cv::putText(result, "LAST INSPECTION: OK", cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                std::cout << "[송신] 판정 결과: OK ('O')" << std::endl;
            } else {
                tx_data = 'N';
                cv::putText(result, "LAST INSPECTION: NG", cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
                std::cout << "[송신] 판정 결과: NG ('N')" << std::endl;
            }

            // STM32 장치로 결과 데이터 1바이트 쓰기
            write(serial_fd, &tx_data, 1);

            // 찰나의 순간에 꼬일 수 있는 시리얼 버퍼 잔여물 청소
            tcflush(serial_fd, TCIOFLUSH);
        }

        // 디스플레이 윈도우 출력
        cv::imshow("Inspection Screen (Interactive Mode)", result);

        if (cv::waitKey(30) == 27) { // ESC 키 입력 시 종료
            break;
        }
    }

    close(serial_fd);
    cap.release();
    cv::destroyAllWindows();
    return 0;
}