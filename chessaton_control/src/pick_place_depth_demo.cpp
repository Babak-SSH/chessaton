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
    grasp_pose.position.x = 0.02;
    grasp_pose.position.y = -0.285;
    grasp_pose.position.z = 0.035;
    geometry_msgs::msg::Pose approach_pose = grasp_pose;
    approach_pose.position.y += 0.04;
    approach_pose.position.z += 0.03;
    geometry_msgs::msg::Pose lift_pose = grasp_pose;
    lift_pose.position.z += 0.1;
    lift_pose.position.y += 0.05;

    geometry_msgs::msg::Pose goal_pose;
    goal_pose.orientation.w = 1;
    goal_pose.position.x = 0.01;
    goal_pose.position.y = 0.22;
    goal_pose.position.z = 0.20;
    geometry_msgs::msg::Pose approach_goal_pose = goal_pose;
    approach_goal_pose.position.z += 0.005;
    geometry_msgs::msg::Pose retreat_pose = goal_pose;
    retreat_pose.position.y -= 0.08;

    // define a collision box
    std::string box_id = "blue_box";

    shape_msgs::msg::SolidPrimitive box_primitive;
    box_primitive.type = box_primitive.BOX;
    box_primitive.dimensions.resize(3);
    box_primitive.dimensions[0] = 0.028;
    box_primitive.dimensions[1] = 0.028;
    box_primitive.dimensions[2] = 0.054;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0;
    box_pose.position.y = -0.30;
    box_pose.position.z = 0.045;

    robot_mp.add_collision_obj(box_primitive, box_pose, box_id);
    robot_mp.allow_collision(box_id, true);

    // open gripper
    robot_mp.move_gripper("hand_open");

    // approach to box
    robot_mp.move_to(approach_pose);

    // move tcp to box
    robot_mp.move_to(grasp_pose);

    // grasp the box
    robot_mp.move_gripper(0.014);

    robot_mp.attach_obj(box_id);

    // lift the box
    robot_mp.move_to(lift_pose);

    // go to goal position
    robot_mp.move_to(approach_goal_pose);
    robot_mp.move_to(goal_pose);

    // open the gripper to release the box
    robot_mp.move_gripper("hand_open");
    robot_mp.detach_obj(box_id);
    robot_mp.remove_obj(box_id);
    robot_mp.move_to(retreat_pose);

    // got to home position
    robot_mp.home();

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}