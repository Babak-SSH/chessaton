#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


struct Position {
    double x, y, z;
    Position(double a, double b, double c) {this->x = a; this->y = b; this->z = c;}
};

// ROS logger
auto const logger = rclcpp::get_logger("move_group_demo");

void move_to(rclcpp::Node::SharedPtr node, Position position) {
    // set planning group (joint model group)
    static const std::string PLANNING_GROUP_ARM = "chessaton_arm";
    static const std::string PLANNING_GROUP_HAND = "chessaton_hand";

    // MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface_arm= MoveGroupInterface(node, PLANNING_GROUP_ARM);
    auto move_group_interface_hand = MoveGroupInterface(node, PLANNING_GROUP_HAND);

    // MoveIt planning scene interface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);


    move_group_interface_arm.setEndEffectorLink("chessaton_gripper");
    const moveit::core::LinkModel* eef_link = move_group_interface_arm.getCurrentState()->getLinkModel("chessaton_gripper");

    // Visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(node, "chessaton_link0", "testp",
                                                        move_group_interface_arm.getRobotModel());

    visual_tools.deleteAllMarkers();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

    // Getting Basic Information
    RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface_arm.getPlanningFrame().c_str());
    RCLCPP_INFO(logger, "End effector link: %s", move_group_interface_arm.getEndEffectorLink().c_str());
    RCLCPP_INFO(logger, "Available Planning Groups:");
    std::copy(move_group_interface_arm.getJointModelGroupNames().begin(), move_group_interface_arm.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

    geometry_msgs::msg::PoseStamped myPose = move_group_interface_arm.getCurrentPose(); 

    RCLCPP_INFO(logger, "%f %f %f", myPose.pose.position.x, myPose.pose.position.y, myPose.pose.position.z);
    moveit::core::RobotStatePtr current_state = move_group_interface_arm.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    move_group_interface_arm.setPlanningTime(60);

    // Planning to a Pose goal

    // Set a target Pose
    // TODO: get the position from a gui or other ros publisher with type geometry_msg
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = position.x;
    pose.position.y = position.y;
    pose.position.z = position.z;

    Eigen::Isometry3d goal_pose = Eigen::Isometry3d::Identity();
    goal_pose.translation().x() = position.x+0.01;
    goal_pose.translation().y() = position.y+0.01;
    goal_pose.translation().z() = position.z+0.02;

    // move_group_interface.setPoseTarget(pose);
    move_group_interface_arm.setPositionTarget(position.x, position.y, position.z);
    move_group_interface_arm.setGoalTolerance(0.002);

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    auto const [success, my_plan] = [&move_group_interface_arm]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface_arm.plan(msg));
      return std::make_pair(ok, msg);
    }();

    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);
    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // We can also visualize the plan as a line with markers in RViz.
    RCLCPP_INFO(logger, "Visualizing plan 1 as trajectory line");
    // visual_tools.publishAxisLabeled(pose, "pose1");
    visual_tools.publishSphere(pose, rvt::ORANGE);
    visual_tools.publishText(goal_pose, "Pose_Goal", rvt::WHITE, rvt::MEDIUM);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, eef_link, joint_model_group, rvt::PURPLE);
    visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

    // Execute the plan
    if(success) {
      move_group_interface_arm.execute(my_plan);
    } else {
      RCLCPP_ERROR(logger, "Planning failed!");
    }

}

int main(int argc, char * argv[]) {
     double x,y,z;
    
    std::cout << "x: ";
    std::cin >> x;
    std::cout << "y: ";
    std::cin >> y;
    std::cout << "z: ";
    std::cin >> z;

    Position position(x, y, z);

   // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
      "move_group_demo",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    //   node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    move_to(node, position);
    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}