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

    geometry_msgs::msg::Pose grasp_pose;
    grasp_pose.orientation.w = 1;
    grasp_pose.position.x = 0;
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
    goal_pose.position.z = 0.19;
    geometry_msgs::msg::Pose approach_goal_pose = goal_pose;
    approach_goal_pose.position.z += 0.005;
    geometry_msgs::msg::Pose retreat_pose = goal_pose;
    retreat_pose.position.y -= 0.08;

    // define a collision box
    std::string table_id = "box_table";

    shape_msgs::msg::SolidPrimitive table_primitive;
    table_primitive.type = table_primitive.BOX;
    table_primitive.dimensions.resize(3);
    table_primitive.dimensions[0] = 0.2;
    table_primitive.dimensions[1] = 0.2;
    table_primitive.dimensions[2] = 0.15;

    geometry_msgs::msg::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 0.0;
    table_pose.position.y = 0.3;
    table_pose.position.z = 0.075;

    robot_mp.add_collision_obj(table_primitive, table_pose, table_id);

    // you can uncomment the lines related to the box object collisions which we try to lift
    // to have it as a collision object in rviz but tha flow of task sometimes will have mismatch 
    // in collision rule and will prevent the hand to make a solution to lift the box therefore
    // sometimes it won't be able to perform the task.

    // define a collision box
    // std::string box_id = "blue_box";

    // shape_msgs::msg::SolidPrimitive box_primitive;
    // box_primitive.type = box_primitive.BOX;
    // box_primitive.dimensions.resize(3);
    // box_primitive.dimensions[0] = 0.026;
    // box_primitive.dimensions[1] = 0.026;
    // box_primitive.dimensions[2] = 0.054;

    // geometry_msgs::msg::Pose box_pose;
    // box_pose.orientation.w = 1.0;
    // box_pose.position.x = 0;
    // box_pose.position.y = -0.22;
    // box_pose.position.z = 0.04493;

    // robot_mp.add_collision_obj(box_primitive, box_pose, box_id);

    // open gripper
    robot_mp.move_gripper("hand_open");

    // approach to box
    robot_mp.move_to(approach_pose);

    // move tcp to box
    // robot_mp.allow_collision(box_id, true);
    robot_mp.move_to(grasp_pose);

    // grasp the box
    robot_mp.move_gripper(0.014);
    // robot_mp.attach_obj(box_id);

    // lift the box
    robot_mp.move_to(lift_pose);

    // go to goal position  
    robot_mp.move_to(approach_goal_pose);
    robot_mp.move_to(goal_pose);
   
    // open the gripper to release the box
    robot_mp.move_gripper("hand_open");
    // robot_mp.detach_obj(box_id);
    robot_mp.move_to(retreat_pose);

    // got to home position
    robot_mp.home();

    robot_mp.remove_obj(table_id);

    executor.remove_node(node); 
    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}