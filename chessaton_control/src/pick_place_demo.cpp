#include "../include/action_handler.hpp"

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

    // geometry_msgs::msg::Point grasp_position;
    // grasp_position.x = 0.18;
    // grasp_position.y = 0;
    // grasp_position.z = 0.04;
    geometry_msgs::msg::Pose grasp_pose;
    grasp_pose.orientation.w = 1;
    grasp_pose.position.x = 0.205;
    grasp_pose.position.y = 0;
    grasp_pose.position.z = 0.04;
    geometry_msgs::msg::Pose lift_pose = grasp_pose;
    lift_pose.position.z += 0.1;
    lift_pose.position.x -= 0.05;

    // geometry_msgs::msg::Point goal_position;
    // goal_position.x = 0.18;
    // goal_position.y = 0;
    // goal_position.z = 0.04;
    geometry_msgs::msg::Pose goal_pose;
    goal_pose.orientation.w = 1;
    goal_pose.position.x = 0.01;
    goal_pose.position.y = 0.22;
    goal_pose.position.z = 0.20;
    geometry_msgs::msg::Pose above_goal_pose = goal_pose;
    above_goal_pose.position.z += 0.05;
    geometry_msgs::msg::Pose go_back_pose = goal_pose;
    go_back_pose.position.y -= 0.08;



    // define a collision box
    std::string object_id = "box1";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.2;
    primitive.dimensions[1] = 0.2;
    primitive.dimensions[2] = 0.15;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.3;
    box_pose.position.z = 0.075;

    robot_mp.add_collision_obj(primitive, box_pose, object_id);

    // open gripper
    robot_mp.move_gripper("hand_open");

    // move tcp to box
    robot_mp.move_to(grasp_pose);

    // grasp the box
    robot_mp.move_gripper(0.014);

    // lift the box
    grasp_pose.position.z += 0.1;
    grasp_pose.position.x -= 0.05;
    robot_mp.move_to(grasp_pose);

    // go to goal position
    robot_mp.move_to(above_goal_pose);
    robot_mp.move_to(goal_pose);

    // open the gripper to release the box
    robot_mp.move_gripper("hand_open");
    robot_mp.move_to(go_back_pose);

    // got to home position
    robot_mp.home();

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}