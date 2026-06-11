#include <opencv2/opencv.hpp>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>
#include <atomic>
#include <thread>
#include <chrono>
#include <sstream>
#include <iomanip>

std::atomic<bool> g_running{true};
std::atomic<bool> g_inspect_requested{false};

int initSerial(const std::string& port, speed_t baudrate) {
    int serial_port = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_port < 0) {
        std::cerr << "시리얼 포트를 열 수 없습니다: " << port << std::endl;
        return -1;
    }

    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "시리얼 속성을 가져오지 못했습니다." << std::endl;
        close(serial_port);
        return -1;
    }

    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_cflag &= ~HUPCL;
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif

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

    tcflush(serial_port, TCIFLUSH);
    return serial_port;
}

bool sendSerialByte(int serial_fd, char value) {
    const ssize_t written = write(serial_fd, &value, 1);
    if (written != 1) {
        return false;
    }

    tcdrain(serial_fd);
    return true;
}

void serialReaderLoop(int serial_fd) {
    while (g_running.load()) {
        pollfd pfd{};
        pfd.fd = serial_fd;
        pfd.events = POLLIN;

        const int poll_result = poll(&pfd, 1, 100);
        if (poll_result <= 0) {
            continue;
        }

        char rx_buf = 0;
        while (true) {
            const ssize_t bytes_read = read(serial_fd, &rx_buf, 1);
            if (bytes_read <= 0) {
                break;
            }

            if (rx_buf == 'S') {
                g_inspect_requested.store(true);
                std::cout << "\n[수신] STM32 요청('S')" << std::endl;
            } else {
                std::cout << "\n[수신] 예상 외 바이트: 0x"
                          << std::hex << (static_cast<unsigned char>(rx_buf) & 0xFF)
                          << std::dec << std::endl;
            }
        }
    }
}

std::string makeCaptureFilename(bool is_ok) {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time_value = std::chrono::system_clock::to_time_t(now);

    std::ostringstream filename;
    filename << "capture_"
             << std::put_time(std::localtime(&time_value), "%Y%m%d_%H%M%S")
             << (is_ok ? "_OK" : "_NG")
             << ".jpg";
    return filename.str();
}

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "카메라를 열 수 없습니다!" << std::endl;
        return -1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    int serial_fd = initSerial("/dev/ttyACM0", B115200);
    if (serial_fd < 0) {
        std::cerr << "통신 포트를 열 수 없어 프로그램을 종료합니다." << std::endl;
        return -1;
    }

    std::cout << "STM32 serial connected. Waiting for 'S'." << std::endl;
    std::cout << "Running... press STM32 button (exit: ESC)" << std::endl;

    std::thread serial_thread(serialReaderLoop, serial_fd);

    cv::Mat frame, blurred, hsv, ycrcb;
    cv::Mat blue_mask, red_mask1, red_mask2, red_mask, total_normal_mask;

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            break;
        }

        cv::Mat result = frame.clone();

        if (g_inspect_requested.exchange(false)) {
            std::cout << "[처리] 비전 검사 프로세스 시작" << std::endl;

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

            cv::GaussianBlur(equalized, blurred, cv::Size(5, 5), 0);
            cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

            cv::Scalar lower_blue = cv::Scalar(95, 30, 20);
            cv::Scalar upper_blue = cv::Scalar(135, 255, 255);
            cv::inRange(hsv, lower_blue, upper_blue, blue_mask);

            cv::inRange(hsv, cv::Scalar(0, 30, 20), cv::Scalar(12, 255, 255), red_mask1);
            cv::inRange(hsv, cv::Scalar(168, 30, 20), cv::Scalar(180, 255, 255), red_mask2);
            cv::bitwise_or(red_mask1, red_mask2, red_mask);

            cv::bitwise_or(blue_mask, red_mask, total_normal_mask);
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
            cv::morphologyEx(total_normal_mask, total_normal_mask, cv::MORPH_CLOSE, kernel);
            cv::morphologyEx(total_normal_mask, total_normal_mask, cv::MORPH_OPEN, kernel);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(total_normal_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            bool is_target_present = false;
            for (const auto& contour : contours) {
                double area = cv::contourArea(contour);

                if (area > 80000 && area < 300000) {
                    is_target_present = true;
                    cv::Rect rect = cv::boundingRect(contour);
                    cv::rectangle(result, rect, cv::Scalar(0, 255, 0), 2);
                }
            }

            char tx_data = 'N';
            if (is_target_present) {
                tx_data = 'O';
                cv::putText(result, "LAST INSPECTION: OK", cv::Point(20, 40),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                std::cout << "[송신] 판정 결과: OK ('O')" << std::endl;
            } else {
                tx_data = 'N';
                cv::putText(result, "LAST INSPECTION: NG", cv::Point(20, 40),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
                std::cout << "[송신] 판정 결과: NG ('N')" << std::endl;
            }

            if (sendSerialByte(serial_fd, tx_data)) {
                std::cout << "[송신] STM32로 '" << tx_data << "' 전송 완료" << std::endl;
            } else {
                std::cerr << "[송신 실패] STM32로 '" << tx_data << "' 전송 실패" << std::endl;
            }

            const std::string filename = makeCaptureFilename(is_target_present);
            if (cv::imwrite(filename, result)) {
                std::cout << "[저장] " << filename << std::endl;
            } else {
                std::cerr << "[저장 실패] " << filename << std::endl;
            }
        }

        cv::imshow("Inspection Screen (Interactive Mode)", result);

        if (cv::waitKey(30) == 27) {
            break;
        }
    }

    g_running.store(false);
    serial_thread.join();
    close(serial_fd);
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
