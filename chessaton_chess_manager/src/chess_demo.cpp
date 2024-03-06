#include "action_handler.hpp"
#include "position.hpp"

int main(int argc, char * argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
      "chess_demo",
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
    geometry_msgs::msg::Pose approach_grasp_pose;
    geometry_msgs::msg::Pose lift_pose;

    geometry_msgs::msg::Pose goal_pose;
    geometry_msgs::msg::Pose approach_goal_pose;
    geometry_msgs::msg::Pose retreat_pose;

    geometry_msgs::msg::Quaternion gripperQ;
    gripperQ.x = 0;
    gripperQ.y = 0.707107;
    gripperQ.z = 0;
    gripperQ.w = 0.707107;

    float graspZ = 0.045;
    float approachZ = 0.06;
    float liftZ = 0.1;
    float goalZ = 0.045;
    float retreatZ = 0.1;
    float graspWidth = 0.003;

    int start = a1;
    int end = h1;

    for(int i=start; i < end; i++) {
        grasp_pose.orientation = gripperQ;
        std::tie(grasp_pose.position.x, grasp_pose.position.y) = SqToPos[i];
        grasp_pose.position.z = graspZ;

        approach_grasp_pose = grasp_pose;
        approach_grasp_pose.position.z = approachZ;

        lift_pose  = grasp_pose;
        lift_pose.position.z = liftZ;

        goal_pose.orientation = gripperQ;
        std::tie(goal_pose.position.x, goal_pose.position.y) = SqToPos[i+1];
        goal_pose.position.z = goalZ;

        approach_goal_pose = goal_pose;
        approach_goal_pose.position.z = approachZ;

        retreat_pose = goal_pose;
        retreat_pose.position.z = retreatZ;

        // open gripper
        robot_mp.move_gripper("hand_open");

        // approach to chess piece 
        robot_mp.move_to(approach_grasp_pose);

        // move tcp to chess piece
        robot_mp.move_to(grasp_pose);

        // grasp the chess piece
        robot_mp.move_gripper(graspWidth);

        // lift the chess piece 
        robot_mp.move_to(lift_pose);

        // go to goal position  
        robot_mp.move_to(approach_goal_pose);
        robot_mp.move_to(goal_pose);
   
        // open the gripper to release the chess piece
        robot_mp.move_gripper("hand_open");

        // retreat from goal pose
        robot_mp.move_to(retreat_pose);
    }

    // got to home position
    robot_mp.home();

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}