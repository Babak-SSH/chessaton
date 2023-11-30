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
    double max_gripper = 0.03; 
    // Chessaton::Position box_position(0.18, 0, 0.04);
    geometry_msgs::msg::Pose grasp_pose;
    grasp_pose.orientation.z = 0;
    grasp_pose.position.x = 0.18;
    grasp_pose.position.y = 0;
    grasp_pose.position.z = 0.04;
    // Chessaton::Position goal_position(0.19, 0.08, 0.04);
    // Chessaton::Position goal_position(0, 0.25, 0.24);
    geometry_msgs::msg::Pose goal_pose;
    goal_pose.orientation.z = 0;
    goal_pose.position.x = 0.18;
    goal_pose.position.y = 0;
    goal_pose.position.z = 0.04;


    // define a collision box
    std::string object_id = "box1";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.2;
    primitive.dimensions[1] = 0.2;
    primitive.dimensions[2] = 0.2;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.3;
    box_pose.position.z = 0.1;

    robot_mp.add_collision_obj(primitive, box_pose, object_id);

    // open gripper
    robot_mp.move_gripper(0.03);

    // move tcp to box
    robot_mp.move_to(grasp_pose);

    // grasp the box
    robot_mp.move_gripper(0.013);

    // lift the box
    grasp_pose.position.z += 0.1;
    grasp_pose.position.x -= 0.05;
    robot_mp.move_to(grasp_pose);

    // go to goal position
    robot_mp.move_to(goal_pose);

    // open the gripper to release the box
    robot_mp.move_gripper(0.03);

    // got to home position
    robot_mp.move_to(grasp_pose);
    // robot_mp.home();

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}