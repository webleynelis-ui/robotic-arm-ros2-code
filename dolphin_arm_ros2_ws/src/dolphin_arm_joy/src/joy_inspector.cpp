#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <map>

class JoyInspector : public rclcpp::Node
{
public:
    JoyInspector() : Node("joy_inspector")
    {
        // 创建订阅者
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyInspector::joy_callback, this, std::placeholders::_1));

        // 设置初始值
        last_axes_ = std::vector<double>(8, 0.0);  // 假设最多有8个轴
        last_buttons_ = std::vector<int>(12, 0);   // 假设最多有12个按钮
        axis_threshold_ = 0.05;
        RCLCPP_INFO(this->get_logger(),"this is joy topic listener");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // 处理轴变化
        for (size_t i = 0; i < msg->axes.size(); ++i)
        {
            if (std::abs(msg->axes[i] - last_axes_[i]) > axis_threshold_)
            {
                // 打印轴变化信息
                RCLCPP_INFO(this->get_logger(), "Axis %zu changed: %.2f -> %.2f", i, last_axes_[i], msg->axes[i]);
                last_axes_[i] = msg->axes[i];
            }
        }

        // 处理按钮变化
        for (size_t i = 0; i < msg->buttons.size(); ++i)
        {
            if (msg->buttons[i] != last_buttons_[i])
            {
                // 打印按钮按下或释放信息
                std::string state = (msg->buttons[i] == 1) ? "PRESSED" : "RELEASED";
                RCLCPP_INFO(this->get_logger(), "Button %zu %s", i, state.c_str());
                last_buttons_[i] = msg->buttons[i];
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    std::vector<double> last_axes_;
    std::vector<int> last_buttons_;
    double axis_threshold_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyInspector>());
    rclcpp::shutdown();
    return 0;
}
