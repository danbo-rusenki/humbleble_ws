/*
 * ROS 2 Humble 向け Pick and Place サンプル
 * amir_mecanum3 (モバイルマニピュレーター) 対応版
 *
 * 【グリッパー制御の変更】
 *   gripper_controller が GripperActionController のため、
 *   MoveGroupInterface::move() ではなく
 *   /gripper_controller/gripper_cmd Action を直接使用する。
 *
 * 【座標系】
 *   world = base_footprint。全ての moveTo の z は LINK0_HEIGHT (0.413) を加算済み。
 *
 * 【グリッパー角度の目安 (URDF の Gripper joint limit より)】
 *   lower = -1.047198 rad (≈ -60°) : 全開
 *   upper =  0.261799 rad (≈ +15°) : 全閉方向
 *   controlGripper(node, gripper_client, -1.047) → 開く
 *   controlGripper(node, gripper_client,  0.0  ) → 閉じる
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;
using MoveGroupInterface     = moveit::planning_interface::MoveGroupInterface;
using PlanningSceneInterface = moveit::planning_interface::PlanningSceneInterface;
using GripperCommand         = control_msgs::action::GripperCommand;
using GripperClient          = rclcpp_action::Client<GripperCommand>;

// -----------------------------------------------------------------------
// 座標系定数
// -----------------------------------------------------------------------

// link0_1 (アーム付け根) の world (base_footprint) 座標系での高さ [m]
// amir_mecanum3.xacro の base_fixed ジョイント: xyz="0.0 0.0 0.413"
const double LINK0_HEIGHT = 0.413;

// link0_1 からテーブル面までの距離 [m]
const double BASE_HEIGHT = 0.248;

// 物体の大きさ (x, y, z) [m]
const std::vector<double> OBJECT_DIMENSION = {0.075, 0.03, 0.13};

// 物体の link0_1 座標系での位置 (衝突オブジェクト用)
const std::vector<double> OBJECT_POSITION_LOCAL = {
  0.4,
  -0.018171 + OBJECT_DIMENSION[1] / 2.0,
  -BASE_HEIGHT + OBJECT_DIMENSION[2] / 2.0
};

// 物体の world (base_footprint) 座標系での位置 (moveTo 用)
const std::vector<double> OBJECT_POSITION_WORLD = {
  OBJECT_POSITION_LOCAL[0],
  OBJECT_POSITION_LOCAL[1],
  OBJECT_POSITION_LOCAL[2] + LINK0_HEIGHT
};

// -----------------------------------------------------------------------
// ユーティリティ
// -----------------------------------------------------------------------

inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

// グリッパーを GripperActionController で制御 (同期)
// position : Gripper joint 目標角度 [rad]  (-1.047 = 全開, 0.0 = 閉じる)
// max_effort: 最大トルク [Nm]
void controlGripper(
  rclcpp::Node::SharedPtr node,
  GripperClient::SharedPtr client,
  double position,
  double max_effort = 10.0)
{
  if (!client->wait_for_action_server(3s)) {
    RCLCPP_ERROR(node->get_logger(), "Gripper action server not available!");
    return;
  }

  auto goal = GripperCommand::Goal();
  goal.command.position   = position;
  goal.command.max_effort = max_effort;

  // ゴール送信
  auto future_handle = client->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(node, future_handle) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to send gripper goal");
    return;
  }
  auto goal_handle = future_handle.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Gripper goal was rejected");
    return;
  }

  // 結果を同期的に待機
  auto result_future = client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Gripper action did not finish");
    return;
  }

  auto result = result_future.get();
  RCLCPP_INFO(node->get_logger(),
    "Gripper -> %.3f rad (reached_goal: %s)",
    position, result.result->reached_goal ? "yes" : "no");
}

// アームを指定姿勢へ移動 (world フレーム基準)
void moveTo(MoveGroupInterface &group,
            double x, double y, double z,
            double roll_deg, double pitch_deg, double yaw_deg,
            double speed_scaling)
{
  tf2::Quaternion q;
  q.setRPY(deg2rad(roll_deg), deg2rad(pitch_deg), deg2rad(yaw_deg));
  geometry_msgs::msg::Pose target;
  target.orientation = tf2::toMsg(q);
  target.position.x  = x;
  target.position.y  = y;
  target.position.z  = z;
  group.setMaxVelocityScalingFactor(speed_scaling);
  group.setPoseTarget(target);
  group.setGoalTolerance(0.01);
  auto result = group.move();
  if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(rclcpp::get_logger("pick_place"), "Motion to target failed");
  }
}

// 衝突オブジェクトをシーンに追加
void addCollisionObjects(PlanningSceneInterface &scene) {
  std::vector<moveit_msgs::msg::CollisionObject> objs(1);
  objs[0].id              = "table2";
  objs[0].header.frame_id = "base_link";
  shape_msgs::msg::SolidPrimitive prim;
  prim.type = prim.BOX;
  prim.dimensions.resize(3);
  prim.dimensions[0] = OBJECT_DIMENSION[0];
  prim.dimensions[1] = OBJECT_DIMENSION[1];
  prim.dimensions[2] = OBJECT_DIMENSION[2];
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x    = OBJECT_POSITION_LOCAL[0];
  pose.position.y    = OBJECT_POSITION_LOCAL[1];
  pose.position.z    = -BASE_HEIGHT - 0.005;
  objs[0].primitives.push_back(prim);
  objs[0].primitive_poses.push_back(pose);
  objs[0].operation = objs[0].ADD;
  scene.applyCollisionObjects(objs);
}

// -----------------------------------------------------------------------
// メイン
// -----------------------------------------------------------------------

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pick_place");

  // グリッパー Action クライアント
  auto gripper_client = rclcpp_action::create_client<GripperCommand>(
    node, "/gripper_controller/gripper_cmd");

  // MoveGroup (グリッパーは MoveGroup から削除、Action で直接制御)
  MoveGroupInterface arm(node, "arm");
  PlanningSceneInterface planning_scene_interface;

  arm.setPoseReferenceFrame("base_footprint");

  arm.setPlanningTime(30.0);

  rclcpp::sleep_for(1s);
  // addCollisionObjects(planning_scene_interface);

  // --- pick ---
  RCLCPP_INFO(node->get_logger(), "Move above object");
  // moveTo(arm,
  //   OBJECT_POSITION_WORLD[0],
  //   OBJECT_POSITION_WORLD[1],
  //   OBJECT_DIMENSION[2] + 0.05 + LINK0_HEIGHT,
  //   -90.0, 0.0, -90.0, 0.6);

  controlGripper(node, gripper_client, -1.047); 

  moveTo(arm, 0.4, 0.4, 0.3, -90.0, 0.0, -90.0, 1.0);

  // moveTo(arm, 0.4, -0.4, 0.3, -90.0, 0.0, 90.0, 0.6);

  moveTo(arm, 0.4, 0.0, 0.4 , 0.0, 0.0, 0.0, 1.0);

  // moveTo(arm, 0.4, -0.4, 0.3, -90.0, -90.0, 90.0, 0.6);


  
  // moveTo(arm, 0.4, 0.0, 0.3, 0.0, 0.0, 90.0, 0.6);



  // RCLCPP_INFO(node->get_logger(), "Open gripper");
  // controlGripper(node, gripper_client, -1.047);   // 全開

  // RCLCPP_INFO(node->get_logger(), "Approach object slowly");
  // moveTo(arm,
  //   OBJECT_POSITION_WORLD[0],
  //   OBJECT_POSITION_WORLD[1],
  //   OBJECT_DIMENSION[2] - 0.05 + LINK0_HEIGHT,
  //   -90.0, 0.0, -90.0, 0.3);

  // RCLCPP_INFO(node->get_logger(), "Close gripper");
  // controlGripper(node, gripper_client, 0.0);      // 閉じる

  // RCLCPP_INFO(node->get_logger(), "Lift object");
  // moveTo(arm,
  //   OBJECT_POSITION_WORLD[0],
  //   OBJECT_POSITION_WORLD[1],
  //   OBJECT_DIMENSION[2] + 0.1 + LINK0_HEIGHT,
  //   -90.0, 0.0, -90.0, 0.3);

  // // 中間姿勢
  // moveTo(arm, 0.0, 0.4, 0.4 + LINK0_HEIGHT, -90.0, 0.0, -90.0, 0.6);

  // // --- place ---
  // RCLCPP_INFO(node->get_logger(), "Move to place location");
  // moveTo(arm,
  //   -OBJECT_POSITION_WORLD[0],
  //   OBJECT_POSITION_WORLD[1],
  //   OBJECT_DIMENSION[2] + 0.4 + LINK0_HEIGHT,
  //   -90.0, 0.0, 90.0, 0.6);

  // RCLCPP_INFO(node->get_logger(), "Place object slowly");
  // moveTo(arm,
  //   -OBJECT_POSITION_WORLD[0],
  //   OBJECT_POSITION_WORLD[1],
  //   OBJECT_DIMENSION[2] + 0.18 + LINK0_HEIGHT,
  //   -90.0, 0.0, 90.0, 0.3);

  // RCLCPP_INFO(node->get_logger(), "Open gripper");
  // controlGripper(node, gripper_client, -1.047);   // 開放

  // RCLCPP_INFO(node->get_logger(), "Retreat");
  // moveTo(arm,
  //   -OBJECT_POSITION_WORLD[0],
  //   OBJECT_POSITION_WORLD[1],
  //   OBJECT_DIMENSION[2] + 0.30 + LINK0_HEIGHT,
  //   -90.0, 0.0, 90.0, 0.3);

  // RCLCPP_INFO(node->get_logger(), "Return to home");
  // moveTo(arm, 0.0, 0.4, 0.4 + LINK0_HEIGHT, 0.0, 0.0, 0.0, 0.6);

  RCLCPP_INFO(node->get_logger(), "Pick and place done");

  rclcpp::shutdown();
  return 0;
}
