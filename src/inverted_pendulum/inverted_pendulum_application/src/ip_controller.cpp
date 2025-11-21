#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <Eigen/Dense>

class InvertedPendulumController : public rclcpp::Node
{
public:
  InvertedPendulumController() : Node("inverted_pendulum_controller")
  {
    // Initialise subscribers, publishers, and parameters
    initializeROS();
    // Set up system matrices
    setupSystemMatrices();
    // Compute LQR gain
    computeLQR();
  }

private:
  // ROS2 members
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // LQR members
  Eigen::Matrix4d A_;
  Eigen::Vector4d B_;
  Eigen::Matrix4d Q_;
  Eigen::Matrix<double, 1, 1> R_;
  Eigen::Matrix<double, 1, 4> K_;
  Eigen::Vector4d state_;

  void initializeROS();
  void setupSystemMatrices();
  void computeLQR();
  void stateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void controlLoop();
};

// class InvertedPendulumController : public rclcpp::Node {
// public:
//     InvertedPendulumController() : Node("ip_controller") {
//         // Initialise controller here
//     }

//     void updateState(double u, double dt);
//     void computeLQR();
//     double computeControl();

// private:
//     Eigen::Vector4d state; // [x, x_dot, theta, theta_dot]
//     Eigen::Matrix4d A; // System matrix
//     Eigen::Vector4d B; // Input matrix
//     Eigen::Matrix4d K; // LQR gain matrix
// };

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InvertedPendulumController>());
  rclcpp::shutdown();
  return 0;
}