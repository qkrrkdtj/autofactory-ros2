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
#include <mutex>
#include <condition_variable>

std::atomic<bool> g_running{true};
std::mutex g_inspect_mutex;
std::condition_variable g_inspect_cv;
bool g_inspect_requested = false;

static void requestInspect()
{
    {
        std::lock_guard<std::mutex> lock(g_inspect_mutex);
        g_inspect_requested = true;
    }
    g_inspect_cv.notify_one();
}

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
                requestInspect();
                std::cout << "\n[수신] STM32 요청('S')" << std::endl;
            } else {
                std::cout << "\n[수신] 예상 외 바이트: 0x"
                          << std::hex << (static_cast<unsigned char>(rx_buf) & 0xFF)
                          << std::dec << std::endl;
            }
        }
    }
}

std::string makeCaptureFilename(char result) {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time_value = std::chrono::system_clock::to_time_t(now);

    std::string suffix;
    if (result == 'R') suffix = "_R";
    else if (result == 'B') suffix = "_B";
    else suffix = "_NG";

    std::ostringstream filename;
    filename << "capture_"
             << std::put_time(std::localtime(&time_value), "%Y%m%d_%H%M%S")
             << suffix
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
    std::cout << "Running... press STM32 button (exit: Ctrl+C)" << std::endl;

    std::thread serial_thread(serialReaderLoop, serial_fd);

    cv::Mat frame, blurred, hsv, ycrcb;
    cv::Mat blue_mask, red_mask1, red_mask2, red_mask;

    while (g_running.load()) {
        {
            std::unique_lock<std::mutex> lock(g_inspect_mutex);
            g_inspect_cv.wait(lock, [] {
                return g_inspect_requested || !g_running.load();
            });
            if (!g_running.load()) {
                break;
            }
            g_inspect_requested = false;
        }

        cap >> frame;
        if (frame.empty()) {
            std::cerr << "프레임을 읽을 수 없습니다." << std::endl;
            continue;
        }

        cv::Mat result = frame.clone();

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

            // CLAHE (Y-channel only) brightens dark bottle interiors without shifting hue,
            // ensuring the full disc area is captured rather than just the bright rim ring.
            cv::GaussianBlur(equalized, blurred, cv::Size(5, 5), 0);
            cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

            // H=95~135: includes the teal outer ring of the blue bottle.
            // With per-color contour detection, the red bottle's teal ring (~35k px²)
            // S>=80 excludes low-saturation shadows (e.g. green container interior, S~20-50).
            // H upper bound 150 includes deep blue/indigo of the bottle interior dome (H~130-145).
            cv::inRange(hsv, cv::Scalar(95, 80, 20), cv::Scalar(150, 255, 255), blue_mask);

            // S=20 lower bound captures dark/desaturated reds (e.g. dark bottle interior)
            cv::inRange(hsv, cv::Scalar(0, 20, 20), cv::Scalar(12, 255, 255), red_mask1);
            cv::inRange(hsv, cv::Scalar(168, 20, 20), cv::Scalar(180, 255, 255), red_mask2);
            cv::bitwise_or(red_mask1, red_mask2, red_mask);

            // Red: 21x21 close bridges the ~15-25px green separator ring between outer rim and inner dome.
            // Blue: small kernel only; RETR_EXTERNAL on the teal ring (donut) returns contourArea =
            //       pi*R_outer^2 (~138k px^2), so no large close is needed to pass the area threshold.
            //       A large kernel would over-expand and merge with nearby left-panel blue elements.
            cv::Mat red_close_kernel  = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(21, 21));
            cv::Mat open_kernel       = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
            cv::morphologyEx(red_mask,  red_mask,  cv::MORPH_CLOSE, red_close_kernel);
            cv::morphologyEx(red_mask,  red_mask,  cv::MORPH_OPEN,  open_kernel);
            cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_CLOSE, open_kernel);
            cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_OPEN,  open_kernel);

            std::vector<std::vector<cv::Point>> red_contours, blue_contours;
            cv::findContours(red_mask,  red_contours,  cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            cv::findContours(blue_mask, blue_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            auto isBottleCap = [](const std::vector<cv::Point>& contour) {
                const double area = cv::contourArea(contour);
                if (area < 50000 || area > 500000) return false;

                // Use convex hull for circularity: the raw contour perimeter is inflated
                // by concavities between the teal ring and inner dome, but the convex hull
                // gives a smooth boundary that yields circularity ~1.0 for a real disc.
                std::vector<cv::Point> hull;
                cv::convexHull(contour, hull);
                const double hull_perimeter = cv::arcLength(hull, true);
                const double hull_area      = cv::contourArea(hull);
                const double circularity = (hull_perimeter > 0)
                    ? (4.0 * CV_PI * hull_area / (hull_perimeter * hull_perimeter)) : 0.0;
                if (circularity < 0.7) return false;

                // Reject tall/wide rectangles: bounding rect must be roughly square
                const cv::Rect br = cv::boundingRect(contour);
                const double aspect = static_cast<double>(std::min(br.width, br.height))
                                    / static_cast<double>(std::max(br.width, br.height));
                if (aspect < 0.5) return false;

                // Reject left/right edge noise by centroid position (mask is NOT clipped
                // so the full disc shape is preserved for the shape checks above).
                const cv::Moments m = cv::moments(contour);
                if (m.m00 == 0) return false;
                const double cx = m.m10 / m.m00;
                return cx >= 100.0 && cx <= 540.0;
            };

            bool is_red_present = false;
            bool is_blue_present = false;
            for (const auto& contour : red_contours) {
                if (isBottleCap(contour)) {
                    is_red_present = true;
                    cv::rectangle(result, cv::boundingRect(contour), cv::Scalar(0, 0, 255), 2);
                }
            }
            for (const auto& contour : blue_contours) {
                if (isBottleCap(contour)) {
                    is_blue_present = true;
                    cv::rectangle(result, cv::boundingRect(contour), cv::Scalar(255, 0, 0), 2);
                }
            }

            char tx_data = 'N';
            if (is_red_present) {
                tx_data = 'R';
                cv::putText(result, "LAST INSPECTION: RED", cv::Point(20, 40),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
                std::cout << "[송신] 판정 결과: RED ('R')" << std::endl;
            } else if (is_blue_present) {
                tx_data = 'B';
                cv::putText(result, "LAST INSPECTION: BLUE", cv::Point(20, 40),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);
                std::cout << "[송신] 판정 결과: BLUE ('B')" << std::endl;
            } else {
                cv::putText(result, "LAST INSPECTION: NG", cv::Point(20, 40),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 128), 2);
                std::cout << "[송신] 판정 결과: NG ('N')" << std::endl;
            }

            if (sendSerialByte(serial_fd, tx_data)) {
                std::cout << "[송신] STM32로 '" << tx_data << "' 전송 완료" << std::endl;
            } else {
                std::cerr << "[송신 실패] STM32로 '" << tx_data << "' 전송 실패" << std::endl;
            }

            const std::string filename = makeCaptureFilename(tx_data);
            if (cv::imwrite(filename, result)) {
                std::cout << "[저장] " << filename << std::endl;
            } else {
                std::cerr << "[저장 실패] " << filename << std::endl;
            }

    }

    g_running.store(false);
    g_inspect_cv.notify_all();
    serial_thread.join();
    close(serial_fd);
    cap.release();
    return 0;
}
