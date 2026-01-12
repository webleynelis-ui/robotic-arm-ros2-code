#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <libserial/SerialPort.h>
#include <sstream>
#include <string>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <future>

using namespace std::chrono_literals;

class ServoJoystickGripper : public rclcpp::Node
{
public:
    ServoJoystickGripper()
        : Node("servo_joystick_gripper"),
          angleA_(90.0), angleB_(15.0), angleC_(165.0), angleD_(0.0),
          last_grip_value_(0.0),
          last_send_time_(std::chrono::steady_clock::now())
    {
        try {
            serial_.Open("/dev/ttyUSB0");
            serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            RCLCPP_INFO(this->get_logger(), "Connected to /dev/ttyUSB0");
        }
        catch (const LibSerial::OpenFailed &) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port!");
            return;
        }

        // QoS：仅保留最新一帧，防止 backlog
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", qos,
            std::bind(&ServoJoystickGripper::joy_callback, this, std::placeholders::_1));

        // 保活定时器（每2秒发送一次，防止ESP32超时复位）
        timer_ = this->create_wall_timer(2s, std::bind(&ServoJoystickGripper::send_angles, this));

        RCLCPP_INFO(this->get_logger(),
                    "🎮 Servo joystick node ready (async write + <A,B,C,D> format)");
    }

private:
    // 手柄输入回调
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() < 3) return;

        double hall_axis = msg->axes[2];   // 假设axes[2]控制夹爪
        double input_min = -0.61;
        double input_max = 0.62;
        double output_min = 0.0;
        double output_max = 30.0;

        hall_axis = std::clamp(hall_axis, input_min, input_max);
        double ratio = (hall_axis - input_min) / (input_max - input_min);
        double target_angle = output_max - ratio * (output_max - output_min);
        target_angle = std::clamp(target_angle, output_min, output_max);

        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send_time_).count();

        if (std::abs(target_angle - last_grip_value_) > 0.5 && elapsed > 50) {
            angleD_ = target_angle;
            last_grip_value_ = target_angle;
            last_send_time_ = now;
            send_angles();  // 立即更新
        }
    }
    void send_angles()
    {
        std::stringstream ss;
        ss << "<"
        << static_cast<int>(angleA_) << ","
        << static_cast<int>(angleB_) << ","
        << static_cast<int>(angleC_) << ","
        << static_cast<int>(angleD_) << ">\n";
        std::string cmd = ss.str();

        (void)std::async(std::launch::async, [this, cmd]() {
            try {
                std::lock_guard<std::mutex> lock(serial_mutex_);
                serial_.Write(cmd);
                serial_.DrainWriteBuffer();
            }
            catch (const LibSerial::NotOpen &) {
                RCLCPP_ERROR(this->get_logger(), "Serial not open!");
            }
        });
    }

    // 成员变量
    LibSerial::SerialPort serial_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex serial_mutex_;

    double angleA_, angleB_, angleC_, angleD_;
    double last_grip_value_;
    std::chrono::steady_clock::time_point last_send_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoJoystickGripper>());
    rclcpp::shutdown();
    return 0;
}
