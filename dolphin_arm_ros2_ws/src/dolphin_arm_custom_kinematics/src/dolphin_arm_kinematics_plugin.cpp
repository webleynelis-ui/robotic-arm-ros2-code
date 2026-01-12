#include <moveit/kinematics_base/kinematics_base.h>
#include <pluginlib/class_list_macros.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>

namespace dolphin_arm_kinematics
{

// ---------- 基础工具 ----------
static inline double deg(double r){ return r * 180.0 / M_PI; }
static inline double rad(double d){ return d * M_PI / 180.0; }
static inline double sgn(double x){ return (x>0)-(x<0); }

// ---------- 逆运动学（角度制） ----------
static bool inverse_operation_deg(double x, double y, double z,
                                  double& outA, double& outB, double& outC)
{
  const double k = 72.0 / 28.0; // 映射系数
  double phi = deg(std::atan2(z, x)); // [-180,180]
  double a = NAN;
  for (int n = -2; n <= 2; ++n)
  {
    double cand = 90.0 - k * (phi + 180.0 * n);
    if (cand >= 0.0 && cand <= 180.0){ a = cand; break; }
  }
  if (std::isnan(a))
  {
    double best = 90.0 - k * phi;
    double bestErr = 1e9;
    for (int n = -4; n <= 4; ++n)
    {
      double cand = 90.0 - k * (phi + 180.0 * n);
      double err = 0.0;
      if (cand < 0.0) err = -cand;
      else if (cand > 180.0) err = cand - 180.0;
      if (err < bestErr){ bestErr = err; best = cand; }
    }
    a = std::clamp(best, 0.0, 180.0);
  }

  auto sq = [](double v){ return v*v; };
  double temp1 = deg(std::atan((y - 65.5) / (-sgn(x) * std::sqrt(x*x + z*z) - 7.0 - 60.0)));
  double num   = (135.0*135.0
               + sq(-sgn(x) * std::sqrt(x*x + z*z) - 7.0 - 60.0)
               + sq(y - 65.5) - 145.0*145.0);
  double den   = (2.0 * 135.0
               * std::sqrt(sq(-sgn(x) * std::sqrt(x*x + z*z) - 7.0 - 60.0)
                         + sq(y - 65.5)));
  double arg2 = num / den;
  arg2 = std::clamp(arg2, -1.0, 1.0);
  double temp2 = deg(std::acos(arg2));
  double b = 180.0 - 69.0 - temp2 - temp1;

  double arg3 = (145.0*145.0 + 135.0*135.0
              - sq(-sgn(x) * std::sqrt(x*x + z*z) - 67.0)
              - sq(y - 65.5)) / (2.0 * 145.0 * 135.0);
  arg3 = std::clamp(arg3, -1.0, 1.0);
  double temp3 = deg(std::acos(arg3));
  double c = 180.0 - (83.5 + (180.0 - 69.0 - temp2 - temp1) - temp3);

  outA = a; outB = b; outC = c;
  return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
}

// ---------- 正运动学（角度制） ----------
static void alg_positive_deg(double a, double b, double c,
                             double& x, double& y, double& z)
{
  double temp = -(135.0 * std::cos(rad(111.0 - b))
               + 145.0 * std::sin(rad((83.5 + b - (180.0 - c)) - b + 21.0))
               + 67.0);
  x = std::cos(rad((a - 90.0) / (72.0 / 28.0))) * temp;
  y = 65.5 + 135.0 * std::sin(rad(111.0 - b))
          - 145.0 * std::cos(rad((83.5 + b - (180.0 - c)) - b + 21.0));
  z = -std::sin(rad((a - 90.0) / (72.0 / 28.0))) * temp;
}

// ---------- 联动约束 ----------
static inline bool check_joint_coupling_deg(double b, double c)
{
  double minC = 140.0 - b;
  double maxC = std::min(180.0, 196.0 - b);
  return (c >= minC) && (c <= maxC);
}

// ==============================================================
class DolphinArmIK : public kinematics::KinematicsBase
{
public:
    bool initialize(const rclcpp::Node::SharedPtr& node,
                    const std::string& group_name,
                    const std::string& base_frame,
                    const std::string& tip_frame,
                    double /*search_discretization*/)
    {
    node_ = node;
    group_name_ = group_name;
    base_frame_ = base_frame;
    tip_link_   = tip_frame;
    initialized_ = true;

    RCLCPP_INFO(node_->get_logger(),
                "✅ DolphinArmIK initialized (Humble-compatible). group='%s', base='%s', tip='%s'",
                group_name.c_str(), base_frame.c_str(), tip_frame.c_str());
    return true;
    }


  // ---------- IK ----------
  bool getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                     const std::vector<double>& /*seed_state*/,
                     std::vector<double>& solution,
                     moveit_msgs::msg::MoveItErrorCodes& error_code,
                     const kinematics::KinematicsQueryOptions& /*options*/) const override
  {
    if (!initialized_)
    {
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      return false;
    }

    const double x = ik_pose.position.x;
    const double y = ik_pose.position.y;
    const double z = ik_pose.position.z;

    double Adeg, Bdeg, Cdeg;
    if (!inverse_operation_deg(x, y, z, Adeg, Bdeg, Cdeg))
    {
      RCLCPP_WARN(node_->get_logger(), "[IK ❌] inverse_operation_deg failed.");
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
      return false;
    }

    if (!check_joint_coupling_deg(Bdeg, Cdeg))
    {
      RCLCPP_WARN(node_->get_logger(),
        "[IK ❌] violates coupling constraint: B=%.2f°, C=%.2f°", Bdeg, Cdeg);
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
      return false;
    }

    RCLCPP_INFO(node_->get_logger(),
      "[IK ✅] Pose(x=%.2f, y=%.2f, z=%.2f) → Joint A=%.2f°, B=%.2f°, C=%.2f°",
      x, y, z, Adeg, Bdeg, Cdeg);

    solution = { rad(Adeg), rad(Bdeg), rad(Cdeg) };
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return true;
  }

  // ---------- FK ----------
  bool getPositionFK(const std::vector<std::string>& link_names,
                     const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::msg::Pose>& poses) const override
  {
    poses.clear();
    if (link_names.size() != 1 || link_names[0] != tip_link_)
      return false;
    if (joint_angles.size() < 3) return false;

    double a = deg(joint_angles[0]);
    double b = deg(joint_angles[1]);
    double c = deg(joint_angles[2]);

    double x, y, z;
    alg_positive_deg(a, b, c, x, y, z);

    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    p.orientation.w = 1.0;
    p.orientation.x = p.orientation.y = p.orientation.z = 0.0;
    poses.push_back(p);
    return true;
  }

  // ---------- MoveIt2 需要的 searchPositionIK 接口 ----------
  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        std::vector<double>& solution,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options) const override
  {
    (void)ik_seed_state; (void)timeout;
    return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
  }

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        const std::vector<double>& consistency_limits,
                        std::vector<double>& solution,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options) const override
  {
    (void)consistency_limits;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, error_code, options);
  }

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        std::vector<double>& solution,
                        const IKCallbackFn& solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options) const override
  {
    bool success = searchPositionIK(ik_pose, ik_seed_state, timeout, solution, error_code, options);
    if (success && solution_callback)
      solution_callback(ik_pose, solution, error_code);
    return success;
  }

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        const std::vector<double>& consistency_limits,
                        std::vector<double>& solution,
                        const IKCallbackFn& solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options) const override
  {
    bool success = searchPositionIK(ik_pose, ik_seed_state, timeout,
                                    consistency_limits, solution, error_code, options);
    if (success && solution_callback)
      solution_callback(ik_pose, solution, error_code);
    return success;
  }

  const std::vector<std::string>& getJointNames() const override
  {
    if (joint_names_.empty())
      joint_names_ = {"joint1", "joint2", "joint3"};
    return joint_names_;
  }

  const std::vector<std::string>& getLinkNames() const override
  {
    if (link_names_.empty())
      link_names_.push_back(tip_link_);
    return link_names_;
  }

private:
  mutable std::vector<std::string> joint_names_;
  mutable std::vector<std::string> link_names_;
  rclcpp::Node::SharedPtr node_;
  std::string group_name_, base_frame_, tip_link_;
  bool initialized_ = false;
};

} // namespace dolphin_arm_kinematics

PLUGINLIB_EXPORT_CLASS(dolphin_arm_kinematics::DolphinArmIK, kinematics::KinematicsBase)
