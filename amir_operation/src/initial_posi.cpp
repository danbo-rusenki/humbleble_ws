#include <rclcpp/rclcpp.hpp>
#include <amir_interfaces/msg/amir_cmd.hpp>
#include <string>

class JointPublisher : public rclcpp::Node
{
public:
  JointPublisher()
  : Node("vis_joint_publisher")
  { 
    rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
    qos_profile.best_effort();
    joint_pub_ = this->create_publisher<amir_interfaces::msg::AmirCmd>("/motor_sub", qos_profile);
    publish_joint_states(); // 一度だけ呼び出す
  }

private:
  void publish_joint_states()
  {
    auto joint = amir_interfaces::msg::AmirCmd();
    joint.angle[0] = -2967.0;
    joint.angle[1] = -740.0;
    joint.angle[2] = 1000.0;
    joint.angle[3] = -1700.0;
    joint.angle[4] = 2748.0;
    joint.angle[5] = -1250.0;
    // joint.angle[0] = 0.0;
    // joint.angle[1] = 0.0;
    // joint.angle[2] = 0.0;
    // joint.angle[3] = 0.0;
    // joint.angle[4] = 0.0;
    // joint.angle[5] = 0.0;
    joint.vel[0] = 400.0;
    joint.vel[1] = 200.0;
    joint.vel[2] = 400.0;
    joint.vel[3] = 400.0;
    joint.vel[4] = 400.0;
    joint.vel[5] = 200.0;
    for (int i = 0; i < 6; ++i) {
    joint_pub_->publish(joint);
    }
    RCLCPP_INFO(this->get_logger(), "🚀Published joint states.🛸");
  }

  rclcpp::Publisher<amir_interfaces::msg::AmirCmd>::SharedPtr joint_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointPublisher>();
  rclcpp::spin_some(node); 
  rclcpp::shutdown();
  return 0;
}