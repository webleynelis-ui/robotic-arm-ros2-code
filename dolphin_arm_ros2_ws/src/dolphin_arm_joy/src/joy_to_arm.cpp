#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <algorithm>
#include <cmath>
#include <vector>
#include <string>

class JoyToMultiVelocity : public rclcpp::Node
{
public:
    JoyToMultiVelocity()
        : Node("joy_to_multi_velocity")
    {
        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyToMultiVelocity::joy_callback, this, std::placeholders::_1));

        traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/arm_controller/joint_trajectory", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&JoyToMultiVelocity::timer_callback, this));

        joint_names_ = {"Fixed_Base_rot_1", "Rotary_Base_rot_2", "BC_Connecting_Rod_rot_3"};
        joint_pos_ = {0.0, 0.0, 0.0};
        neutral_ = {0.0, 0.0, 0.0};
        direction_ = {0, 0, 0};
        calibrated_ = false;

        joint_min_ = {-1.570796, -1.308997, -0.698132};
        joint_max_ = {1.570796,  0.349066,  0.959931};

        // 固定步长，可自行调整
        base_step_ = 0.02;  // 弧度制；你可以改为 0.01、0.03 试效果

        deadzone_ = 0.12;

        axis_map_[0] = 3;
        axis_map_[1] = 1;
        axis_map_[2] = 0;

        sign_map_[0] = -1;
        sign_map_[1] =  1;
        sign_map_[2] = -1;

        RCLCPP_INFO(this->get_logger(), "✅ Joy control initialized (fixed step mode, step=%.3f)", base_step_);
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() < 4 || msg->buttons.size() < 4) return;

        if (msg->buttons[2] == 1)
        {
            joint_pos_ = {0.0, 0.0, 0.0};
            for (size_t i = 0; i < 3; ++i)
                neutral_[i] = msg->axes[axis_map_[i]];
            calibrated_ = true;
            RCLCPP_WARN(this->get_logger(), "Reset & recalibrated neutral: [%.3f, %.3f, %.3f]",
                        neutral_[0], neutral_[1], neutral_[2]);
            return;
        }

        if (!calibrated_)
        {
            for (size_t i = 0; i < 3; ++i)
                neutral_[i] = msg->axes[axis_map_[i]];
            calibrated_ = true;
            RCLCPP_INFO(this->get_logger(), "Neutral calibrated at [%.3f, %.3f, %.3f]",
                        neutral_[0], neutral_[1], neutral_[2]);
            return;
        }

        // === 固定步长模式 ===
        for (size_t i = 0; i < 3; ++i)
        {
            double raw = sign_map_[i] * (msg->axes[axis_map_[i]] - neutral_[i]);
            if (std::fabs(raw) < deadzone_)
                direction_[i] = 0;
            else
                direction_[i] = (raw > 0) ? 1 : -1;
        }
    }

    void timer_callback()
    {
        if (!calibrated_) return;

        // === 更新角度 ===
        for (size_t i = 0; i < 3; ++i)
        {
            joint_pos_[i] += direction_[i] * base_step_;
            joint_pos_[i] = std::clamp(joint_pos_[i], joint_min_[i], joint_max_[i]);
        }

        // === 【新增】机械联动约束 ===
        // 当第二个关节(B)转动时，第三个关节(C)应同步变化
        // 例如：C = B + offset   （或差值方式：ΔC = ΔB）
        // 为简单起见，这里设 ΔC = ΔB
        static double last_B = joint_pos_[1];

        double deltaB = joint_pos_[1] - last_B;
        joint_pos_[2] -= deltaB;  // J3 跟随 J2 变化
        joint_pos_[2] = std::clamp(joint_pos_[2], joint_min_[2], joint_max_[2]);

        last_B = joint_pos_[1];

        // === 发布 JointTrajectory ===
        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = this->get_clock()->now();
        traj.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = joint_pos_;
        point.time_from_start = rclcpp::Duration::from_seconds(0.1);
        traj.points.push_back(point);

        traj_pub_->publish(traj);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "🦾 J1=%.3f | J2=%.3f | J3=%.3f (linked) | step=%.3f",
            joint_pos_[0], joint_pos_[1], joint_pos_[2], base_step_);
    }


    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> joint_names_;
    std::vector<double> joint_pos_;
    std::vector<double> neutral_;
    std::vector<int> direction_;
    std::vector<double> joint_min_, joint_max_;
    bool calibrated_;
    double deadzone_;
    double base_step_;
    int axis_map_[3];
    int sign_map_[3];
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToMultiVelocity>());
    rclcpp::shutdown();
    return 0;
}
