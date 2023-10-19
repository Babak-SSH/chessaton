#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char * argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");
//   node->set_parameter(rclcpp::Parameter("use_sim_time", true));

 // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

    // Next step goes here
    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "chessaton_arm");

  // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup("chessaton_arm");

    geometry_msgs::msg::PoseStamped myPose = move_group_interface.getCurrentPose(); 

    RCLCPP_INFO(logger, "%f %f %f", myPose.pose.position.x, myPose.pose.position.y, myPose.pose.position.z);
    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // Visualization
    // ^^^^^^^^^^^^^
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(node, "chessaton_link0", "testp",
                                                        move_group_interface.getRobotModel());

    visual_tools.deleteAllMarkers();

    /* Remote control is an introspection tool that allows users to step through a high level script */
    /* via buttons and keyboard shortcuts in RViz */
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

 // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(logger, "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(logger, "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(), move_group_interface.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

    move_group_interface.setPlanningTime(60);
    // Set a target Pose
    // auto const target_pose = []{
      geometry_msgs::msg::Pose msg;
    //   msg.orientation.w = 1.0;
    //   msg.position.x = myPose.pose.position.x - 0.01;
    //   msg.position.y = myPose.pose.position.y - 0.01;
      msg.position.z = myPose.pose.position.z;

      msg.position.x = 0.040087;
      msg.position.y = 0.12889;
    //   msg.position.z = 0.194;

    //   return msg;
    // }();
    // move_group_interface.setPoseTarget(msg);
    // move_group_interface.setPositionTarget(msg.position.x, msg.position.y, msg.position.z);
    move_group_interface.setPositionTarget(0.03, 0.1, 0.15);
    move_group_interface.setGoalTolerance(0.01);

    // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//   bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//   RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Create a plan to that target pose
    auto const [success, my_plan] = [&move_group_interface]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

// joint_group_positions[0] = -1.0;  // radians
// move_group_interface.setJointValueTarget(joint_group_positions);
// moveit::planning_interface::MoveGroupInterface::Plan my_plan;
// bool success =  static_cast<bool>(move_group_interface.plan(my_plan));
// RCLCPP_INFO(logger, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

/* Sleep to give Rviz time to visualize the plan. */
sleep(5.0);
    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // We can also visualize the plan as a line with markers in RViz.
    RCLCPP_INFO(logger, "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(msg, "pose1");
    visual_tools.publishSphere(msg, rvt::PURPLE);
    visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
/* Sleep to give Rviz time to visualize the plan. */
sleep(5.0);

    // Execute the plan
    if(success) {
      move_group_interface.execute(my_plan);
    } else {
      RCLCPP_ERROR(logger, "Planning failed!");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}