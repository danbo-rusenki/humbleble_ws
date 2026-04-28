// // #include <rclcpp/rclcpp.hpp>
// // #include <trajectory_msgs/msg/joint_trajectory.hpp>
// // #include <chrono>

// // using namespace std::chrono_literals;

// // class JointTrajectoryPublisher : public rclcpp::Node
// // {
// // public:
// //   JointTrajectoryPublisher()
// //   : Node("vis_joint_publisher")
// //   { 
// //     // YAMLの controller_name: "arm_controller" に合わせたトピック名
// //     joint_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
// //       "/arm_controller/joint_trajectory", 10);

// //     // Controller側のSubscriber準備ができるまで待機（1秒後に送信）
// //     timer_ = this->create_wall_timer(
// //       1000ms, std::bind(&JointTrajectoryPublisher::publish_joint_states, this));
// //   }

// // private:
// //   void publish_joint_states()
// //   {
// //     auto msg = trajectory_msgs::msg::JointTrajectory();
    
// //     // YAMLの joints リストに合わせて5軸のジョイント名を設定
// //     msg.joint_names = {"Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5"};

// //     trajectory_msgs::msg::JointTrajectoryPoint point;
    
// //     // YAMLの pos1 の値（ラジアン）を設定
// //     point.positions = {0.00, 2.3, -2.3, -0.50, 2.76};
    
// //     // YAMLでコメントアウトされていた安全な速度設定（0.1 rad/s）を適用
// //     point.velocities = {0.1, 0.1, 0.1, 0.1, 0.1};

// //     // YAMLの wait_sec_between_publish を参考に、到達時間を5秒に設定
// //     point.time_from_start.sec = 5;
// //     point.time_from_start.nanosec = 0;

// //     msg.points.push_back(point);

// //     joint_pub_->publish(msg);
// //     RCLCPP_INFO(this->get_logger(), "🚀Published pos1 trajectory to /arm_controller.🛸");

// //     // 初期位置への送信は1回で十分なため、送信後にタイマーを停止
// //     timer_->cancel();
// //   }

// //   rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
// //   rclcpp::TimerBase::SharedPtr timer_;
// // };

// // int main(int argc, char ** argv)
// // {
// //   rclcpp::init(argc, argv);
// //   auto node = std::make_shared<JointTrajectoryPublisher>();
  
// //   // スピンを維持して、ROS 2のイベントループを生かしておく
// //   rclcpp::spin(node); 
  
// //   rclcpp::shutdown();
// //   return 0;
// // }

// #include <rclcpp/rclcpp.hpp>
// #include <trajectory_msgs/msg/joint_trajectory.hpp>
// #include <chrono>

// using namespace std::chrono_literals;

// class JointTrajectoryPublisher : public rclcpp::Node
// {
// public:
//   JointTrajectoryPublisher()
//   : Node("vis_joint_publisher")
//   { 
//     // アーム用コントローラー
//     arm_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
//       "/arm_controller/joint_trajectory", 10);

//     // グリッパー用コントローラー（追加）
//     gripper_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
//       "/gripper_controller/joint_trajectory", 10);

//     timer_ = this->create_wall_timer(
//       1000ms, std::bind(&JointTrajectoryPublisher::publish_joint_states, this));
//   }

// private:
//   void publish_joint_states()
//   {
//     // --- アームの送信 ---
//     auto arm_msg = trajectory_msgs::msg::JointTrajectory();
//     arm_msg.joint_names = {"Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5"};

//     trajectory_msgs::msg::JointTrajectoryPoint arm_point;
//     arm_point.positions  = {0.00, 2.3, -2.3, -0.50, 2.76};
//     arm_point.velocities = {0.1, 0.1, 0.1, 0.1, 0.1};
//     arm_point.time_from_start.sec    = 5;
//     arm_point.time_from_start.nanosec = 0;

//     arm_msg.points.push_back(arm_point);
//     arm_pub_->publish(arm_msg);
//     RCLCPP_INFO(this->get_logger(), "🚀 Published arm pos1 to /arm_controller.");

//     // --- グリッパーの送信（追加） ---
//     auto gripper_msg = trajectory_msgs::msg::JointTrajectory();
//     // ※ URDFのグリッパージョイント名に合わせて変更してください
//     gripper_msg.joint_names = {"Gripper"};

//     trajectory_msgs::msg::JointTrajectoryPoint gripper_point;
//     gripper_point.positions  = {-0.3};
//     gripper_point.velocities = {0.09};          
//     gripper_point.time_from_start.sec    = 5;
//     gripper_point.time_from_start.nanosec = 0;

//     gripper_msg.points.push_back(gripper_point);
//     gripper_pub_->publish(gripper_msg);
//     RCLCPP_INFO(this->get_logger(), "🤏 Published gripper angle to /gripper_controller.");

//     timer_->cancel();
//   }

//   rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;
//   rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
//   rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<JointTrajectoryPublisher>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }


#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <chrono>

using namespace std::chrono_literals;
using GripperCommand = control_msgs::action::GripperCommand;

class JointTrajectoryPublisher : public rclcpp::Node
{
public:
  JointTrajectoryPublisher()
  : Node("vis_joint_publisher")
  {
    // アーム用（トピック）
    arm_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", 10);

    // グリッパー用（Action Client）
    gripper_client_ = rclcpp_action::create_client<GripperCommand>(
      this, "/gripper_controller/gripper_cmd");

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&JointTrajectoryPublisher::publish_joint_states, this));
  }

private:
  void publish_joint_states()
  {
    // --- アームの送信（変更なし）---
    auto arm_msg = trajectory_msgs::msg::JointTrajectory();
    arm_msg.joint_names = {"Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5"};

    trajectory_msgs::msg::JointTrajectoryPoint arm_point;
    arm_point.positions  = {0.00, 2.3, -2.3, -0.50, 2.76};
    arm_point.velocities = {0.1, 0.1, 0.1, 0.1, 0.1};
    arm_point.time_from_start.sec     = 5;
    arm_point.time_from_start.nanosec = 0;
    arm_msg.points.push_back(arm_point);
    arm_pub_->publish(arm_msg);
    RCLCPP_INFO(this->get_logger(), "🚀 Published arm pos1 to /arm_controller.");

    // --- グリッパーの送信（Action）---
    if (!gripper_client_->wait_for_action_server(2s)) {
      RCLCPP_ERROR(this->get_logger(), "❌ gripper_cmd action server not available!");
      timer_->cancel();
      return;
    }

    auto goal = GripperCommand::Goal();
    goal.command.position   = -0.3;   // ← ここで開閉角度を指定（ラジアン）
    goal.command.max_effort = 10.0;  // ← 最大トルク（Nm）

    gripper_client_->async_send_goal(goal);
    RCLCPP_INFO(this->get_logger(), "🤏 Sent gripper goal: %.2f rad", goal.command.position);

    timer_->cancel();
  }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointTrajectoryPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}