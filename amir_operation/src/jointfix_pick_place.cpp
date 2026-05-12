/*
 * ROS 2 Humble / MoveIt2 向け Pick and Place
 * amir_mecanum3 (モバイルマニピュレーター) 対応版
 * * 【修正内容】: 
 * 1. Joint_4 を特定の角度に固定
 * 2. 姿勢(RPY)の制約を解除し、位置(X, Y, Z)のみを指定して移動
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

using namespace std::chrono_literals;
using MoveGroupInterface    = moveit::planning_interface::MoveGroupInterface;
using PlanningSceneInterface = moveit::planning_interface::PlanningSceneInterface;
using GripperCommand         = control_msgs::action::GripperCommand;
using GripperClient          = rclcpp_action::Client<GripperCommand>;

// ═══════════════════════════════════════════════════════════════
// 設定パラメータ
// ═══════════════════════════════════════════════════════════════

// Joint_4 (手首ピッチ) を固定する角度 [rad]
// -1.57 (約-90度) にすると、J2/J3が垂直に近いとき手先が水平を向きやすくなります
const double FIXED_JOINT_4_VALUE = -1.0; 

// 物体情報
const double OBJ_X = 0.45;
const double OBJ_Y = 0.0;
const double OBJ_Z = 0.05; // 物体の中心高さ
const double APPROACH_HEIGHT = 0.15; // アプローチ時の上方オフセット

// 配置位置
const double PLACE_X = 0.4;
const double PLACE_Y = 0.3;
const double PLACE_Z = 0.15;

// グリッパー
const double GRIPPER_OPEN  = -1.0;
const double GRIPPER_CLOSE = 0.1;

// ───────────────────────────────────────────────────────────────
// グリッパー制御関数 (前回同様)
// ───────────────────────────────────────────────────────────────
void controlGripper(rclcpp::Node::SharedPtr node, GripperClient::SharedPtr client, double position) {
    if (!client->wait_for_action_server(5s)) return;
    auto goal = GripperCommand::Goal();
    goal.command.position = position;
    goal.command.max_effort = 50.0;
    auto gh = client->async_send_goal(goal).get();
    if (gh) client->async_get_result(gh).get();
}

// ───────────────────────────────────────────────────────────────
// 【改良版】位置指定 + Joint_4 固定移動関数
// ───────────────────────────────────────────────────────────────
bool moveToPosition(MoveGroupInterface &group, 
                    double x, double y, double z, 
                    double speed_scaling) 
{
    group.setStartStateToCurrentState();

    // 1. Joint_4 の拘束条件を作成
    moveit_msgs::msg::Constraints constraints;
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = "Joint_4"; // xacroの名称に合わせる
    jc.position = FIXED_JOINT_4_VALUE;
    jc.tolerance_above = 0.01; // 許容誤差 約0.5度
    jc.tolerance_below = 0.01;
    jc.weight = 1.0;
    constraints.joint_constraints.push_back(jc);

    // 拘束をMoveGroupに適用
    group.setPathConstraints(constraints);

    // 2. 位置のみを目標に設定 (姿勢は自由)
    group.setPositionTarget(x, y, z);
    group.setGoalPositionTolerance(0.01); // 1cm精度の許容誤差

    group.setMaxVelocityScalingFactor(speed_scaling);
    group.setMaxAccelerationScalingFactor(speed_scaling * 0.5);

    RCLCPP_INFO(rclcpp::get_logger("pick_place"), 
                "Moving to Position (X:%.3f, Y:%.3f, Z:%.3f) with Joint_4 fixed at %.2f", 
                x, y, z, FIXED_JOINT_4_VALUE);

    auto result = group.move();

    // 実行後に拘束をクリア（重要：これを忘れると次回の計画に影響する）
    group.clearPathConstraints();

    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("pick_place"), "Move FAILED (Code: %d)", result.val);
        return false;
    }
    return true;
}

// ───────────────────────────────────────────────────────────────
// メインシーケンス
// ───────────────────────────────────────────────────────────────
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.parameter_overrides({{"use_sim_time", true}});
    auto node = rclcpp::Node::make_shared("fixed_j4_pick_place", node_options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    auto gripper_client = rclcpp_action::create_client<GripperCommand>(node, "/gripper_controller/gripper_cmd");
    MoveGroupInterface arm(node, "arm");
    arm.setPoseReferenceFrame("base_footprint");
    arm.setPlanningTime(10.0);

    // シーケンス開始
    RCLCPP_INFO(node->get_logger(), "=== 固定Joint_4モード Pick & Place 開始 ===");

    // 1. 初期化
    controlGripper(node, gripper_client, GRIPPER_OPEN);

    // 2. アプローチ（物体の上方）
    if (!moveToPosition(arm, OBJ_X, OBJ_Y, OBJ_Z + APPROACH_HEIGHT, 0.5)) {
        goto shutdown;
    }

    // 3. 降下（把持位置）
    if (!moveToPosition(arm, OBJ_X, OBJ_Y, OBJ_Z, 0.2)) {
        goto shutdown;
    }

    // 4. 把持
    RCLCPP_INFO(node->get_logger(), "把持中...");
    controlGripper(node, gripper_client, GRIPPER_CLOSE);
    rclcpp::sleep_for(1s);

    // 5. 持ち上げ
    if (!moveToPosition(arm, OBJ_X, OBJ_Y, OBJ_Z + APPROACH_HEIGHT, 0.3)) {
        goto shutdown;
    }

    // 6. 移動（配置位置の上方）
    if (!moveToPosition(arm, PLACE_X, PLACE_Y, PLACE_Z + APPROACH_HEIGHT, 0.5)) {
        goto shutdown;
    }

    // 7. 配置（降下）
    if (!moveToPosition(arm, PLACE_X, PLACE_Y, PLACE_Z, 0.2)) {
        goto shutdown;
    }

    // 8. 開放
    controlGripper(node, gripper_client, GRIPPER_OPEN);
    RCLCPP_INFO(node->get_logger(), "配置完了。");

    // 9. 退避
    moveToPosition(arm, PLACE_X, PLACE_Y, PLACE_Z + APPROACH_HEIGHT, 0.3);

shutdown:
    RCLCPP_INFO(node->get_logger(), "=== シーケンス終了 ===");
    rclcpp::shutdown();
    spinner.join();
    return 0;
}