#include "../include/action_handler.hpp"
#include <unistd.h>

int main(int argc, char * argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
      "pick_place_demo",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    // we use sim_time for simulation in gazebo
    rclcpp::Parameter sim_time_param("use_sim_time", true);
    node->set_parameter(sim_time_param);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    Chessaton::ActionHandler robot_mp(node);

    geometry_msgs::msg::Pose grasp_pose;
    grasp_pose.orientation.w = 1;
    grasp_pose.position.x = 0;
    grasp_pose.position.y = -0.205;
    grasp_pose.position.z = 0.04;
    geometry_msgs::msg::Pose lift_pose = grasp_pose;
    lift_pose.position.z += 0.1;
    lift_pose.position.y += 0.05;

    geometry_msgs::msg::Pose goal_pose;
    goal_pose.orientation.w = 1;
    goal_pose.position.x = 0.01;
    goal_pose.position.y = 0.22;
    goal_pose.position.z = 0.20;
    geometry_msgs::msg::Pose approach_goal_pose = goal_pose;
    approach_goal_pose.position.z += 0.05;
    geometry_msgs::msg::Pose retreat_pose = goal_pose;
    retreat_pose.position.y -= 0.08;

    // open gripper
    robot_mp.move_gripper("hand_open");

    // move tcp to box
    robot_mp.move_to(grasp_pose);

    // grasp the box
    robot_mp.move_gripper(0.014);

    // lift the box
    robot_mp.move_to(lift_pose);

    // go to goal position
    robot_mp.move_to(approach_goal_pose);
    robot_mp.move_to(goal_pose);

    // open the gripper to release the box
    robot_mp.move_gripper("hand_open");
    robot_mp.move_to(retreat_pose);

    // got to home position
    robot_mp.home();

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}