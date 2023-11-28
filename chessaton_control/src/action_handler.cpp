#include "../include/action_handler.hpp"

#include <iostream>


namespace Chessaton {

    rclcpp::Logger logger = rclcpp::get_logger("ActionHandler");
    ActionHandler::ActionHandler(rclcpp::Node::SharedPtr newNode) {
        node = newNode;
        move_group_interface_arm = new moveit::planning_interface::MoveGroupInterface(node, PLANNING_GROUP_ARM);
        move_group_interface_hand = new moveit::planning_interface::MoveGroupInterface(node, PLANNING_GROUP_HAND);
    }

    void ActionHandler::visualize_path(moveit::planning_interface::MoveGroupInterface::Plan plan, 
                                        const moveit::core::JointModelGroup* joint_model_group, const moveit::core::LinkModel* eef_link,
                                            Position pos) {
        // Visualization
        namespace rvt = rviz_visual_tools;
        moveit_visual_tools::MoveItVisualTools visual_tools(node, "chessaton_link0", "move_group_visual",
                                                            move_group_interface_arm->getRobotModel());

        visual_tools.deleteAllMarkers();

        Eigen::Isometry3d goal_pose = Eigen::Isometry3d::Identity();
        goal_pose.translation().x() = pos.x+0.01;
        goal_pose.translation().y() = pos.y+0.01;
        goal_pose.translation().z() = pos.z+0.02;

        // Set a target Pose
        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = pos.x;
        pose.position.y = pos.y;
        pose.position.z = pos.z;

        // Visualizing plans
        visual_tools.publishSphere(pose, rvt::ORANGE);
        visual_tools.publishText(goal_pose, "Pose_Goal", rvt::WHITE, rvt::MEDIUM);
        visual_tools.publishTrajectoryLine(plan.trajectory_, eef_link, joint_model_group, rvt::PURPLE);
        visual_tools.trigger();
        /* Sleep to give Rviz time to visualize the plan. */
        sleep(2.0);
    }

    bool ActionHandler::move_to(Position pos) {
        move_group_interface_arm->setEndEffectorLink("chessaton_gripper");
        move_group_interface_arm->setPlanningTime(60);

        move_group_interface_arm->setPositionTarget(pos.x, pos.y, pos.z);
        move_group_interface_arm->setGoalTolerance(0.002);

        const moveit::core::JointModelGroup* joint_model_group = move_group_interface_arm->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
        const moveit::core::LinkModel* eef_link = move_group_interface_arm->getCurrentState()->getLinkModel("chessaton_gripper");

        // Getting Basic Information
        RCLCPP_INFO(logger, "Arm Planning frame: %s", move_group_interface_arm->getPlanningFrame().c_str());
        RCLCPP_INFO(logger, "End effector link: %s", move_group_interface_arm->getEndEffectorLink().c_str());
        geometry_msgs::msg::PoseStamped myPose = move_group_interface_arm->getCurrentPose(); 
        RCLCPP_INFO(logger, "Initial pos: %f %f %f", myPose.pose.position.x, myPose.pose.position.y, myPose.pose.position.z);

        // plan the move
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        auto const success = static_cast<bool>(move_group_interface_arm->plan(my_plan));

        ActionHandler::visualize_path(my_plan, joint_model_group, eef_link, pos);

        // Execute the plan
        if(success) {
            move_group_interface_arm->execute(my_plan);
        } else {
            RCLCPP_ERROR(logger, "Planning failed!");
        }

        return success;
    }

} // namespace chessaton